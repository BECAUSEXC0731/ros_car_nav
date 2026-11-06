#!/usr/bin/python3
"""离线复现 lidar_loc 的评分/爬山逻辑（与 C++ 实现一一对应），用于排查跳变。"""
import math
import sys

import numpy as np
from scipy import ndimage
import yaml as pyyaml


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, idx = [], 0
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] != b'\n':
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    idx += 1
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)


yaml_path = sys.argv[1]
meta = pyyaml.safe_load(open(yaml_path))
img = read_pgm(str(yaml_path).rsplit('/', 1)[0] + '/' + meta['image'])
res = float(meta['resolution'])
ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

occ = img < 100
unknown = (img >= 200) & (img < 250)
occ_f = occ[::-1, :]
unk_f = unknown[::-1, :]
h, w = occ_f.shape
grid = np.full((h, w), -1, dtype=np.int8)
grid[~occ_f & ~unk_f] = 0
grid[occ_f] = 100

# ---- 与 C++ buildSearchField 相同 ----
rows, cols = np.where(occ_f)
xmin, xmax = cols.min(), cols.max()
ymin, ymax = rows.min(), rows.max()
R_M = 2.5
margin = max(1, round(R_M / res))
x0 = max(0, xmin - margin)
y0 = max(0, ymin - margin)
x1 = min(w - 1, xmax + margin)
y1 = min(h - 1, ymax + margin)
occ_crop = occ_f[y0:y1 + 1, x0:x1 + 1].astype(np.uint8) * 255
dist = ndimage.distance_transform_edt(occ_crop == 0)
radius_px = max(1.0, R_M / res)
field = np.clip(255.0 * np.maximum(0.0, 1.0 - dist / radius_px), 0, 255).astype(np.uint8)
roi_x, roi_y = x0, y0
print(f'field {field.shape} roi=({roi_x},{roi_y}) obstacle bbox x[{xmin},{xmax}] y[{ymin},{ymax}]')

# ---- 真值位姿 ----
free_mask = grid == 0
BIG = 1e6
up = np.full((h, w), BIG, dtype=np.float64)
for r in range(h):
    prev = up[r - 1] if r > 0 else np.full(w, BIG)
    up[r] = np.where(free_mask[r], np.minimum(prev + 1, BIG), 0)
down = np.full((h, w), BIG, dtype=np.float64)
for r in range(h - 1, -1, -1):
    nxt = down[r + 1] if r < h - 1 else np.full(w, BIG)
    down[r] = np.where(free_mask[r], np.minimum(nxt + 1, BIG), 0)
clearance = np.minimum(up, down) * res
clearance[~free_mask] = 0
r0, c0 = np.unravel_index(int(np.argmax(clearance)), clearance.shape)
truth = (ox + (c0 + 0.5) * res, oy + (r0 + 0.5) * res, 0.0)
print(f'truth = {truth}')


def raycast(x, y, a, max_r=8.0):
    dx, dy = math.cos(a), math.sin(a)
    step = res * 0.5
    rng = 0.0
    while rng < max_r:
        c = int((x + dx * rng - ox) / res)
        r = int((y + dy * rng - oy) / res)
        if not (0 <= r < h and 0 <= c < w) or grid[r, c] != 0:
            return rng
        rng += step
    return max_r


N = 360
pts = []
for i in range(N):
    a = -math.pi + i * math.radians(1)
    r = raycast(truth[0], truth[1], a)
    pts.append((r * math.cos(a), r * math.sin(a)))
pts = np.array(pts)


def score(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    xm = c * pts[:, 0] - s * pts[:, 1] + x
    ym = s * pts[:, 0] + c * pts[:, 1] + y
    col = np.rint((xm - ox) / res).astype(int) - roi_x
    row = np.rint((ym - oy) / res).astype(int) - roi_y
    ok = (col >= 0) & (col < field.shape[1]) & (row >= 0) & (row < field.shape[0])
    return float(field[row[ok], col[ok]].sum())


def match(x, y, yaw, max_iter=400):
    step = res
    yaw_step = math.radians(1.0)
    offs = [(0, 0), (step, 0), (-step, 0), (0, step), (0, -step)]
    best = score(x, y, yaw)
    for _ in range(max_iter):
        bc, bdx, bdy, bdyaw = best, 0.0, 0.0, 0.0
        for ox_, oy_ in offs:
            for dyaw in (0.0, yaw_step, -yaw_step):
                s = score(x + ox_, y + oy_, yaw + dyaw)
                if s > bc:
                    bc, bdx, bdy, bdyaw = s, ox_, oy_, dyaw
        if bdx == 0 and bdy == 0 and bdyaw == 0:
            break
        x, y, yaw = x + bdx, y + bdy, yaw + bdyaw
        best = bc
    return x, y, yaw, best / (255.0 * len(pts))


print(f'真值处得分 {score(*truth) / (255.0 * len(pts)):.4f}')
x, y, yaw = 2.35, -0.62, 0.17
for k in range(6):
    sx, sy, syaw = x, y, yaw
    x, y, yaw, q = match(x, y, yaw)
    print(f'帧{k}: ({sx:.2f},{sy:.2f},{math.degrees(syaw):5.1f}°) '
          f'-> ({x:.3f},{y:.3f},{math.degrees(yaw):6.1f}°) q={q:.4f} '
          f'err={math.hypot(x - truth[0], y - truth[1]):.3f}m')
