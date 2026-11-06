#!/usr/bin/python3
"""离线分析 smear_test bag，回答三个问题：
  1) 去畸变(scan_cloud)相对原始 scan_ts 到底移了多少点（旋转时应该明显，静止时应≈0）
  2) 代价地图致命格是"贴墙"(被点云解释)还是"糊"(解释率低)
  3) map->odom 的逐帧抖动
用法：/usr/bin/python3 tools/analyze_bag.py /tmp/smear_test
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, OccupancyGrid

bag = sys.argv[1] if len(sys.argv) > 1 else '/tmp/smear_test'

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('', ''))

scans = []     # (t, LaserScan)
clouds = []    # (t, list[(x,y)])
tfs = []       # (t, TransformStamped)
odoms = []     # (t, Odometry)
maps = []      # (t, OccupancyGrid)

while reader.has_next():
    topic, data, t = reader.read_next()
    ts = t * 1e-9  # bag 返回纳秒
    if topic == '/scan_ts':
        scans.append((ts, deserialize_message(data, LaserScan)))
    elif topic == '/scan_cloud':
        m = deserialize_message(data, PointCloud2)
        pts = [(p[0], p[1]) for p in pc2.read_points(m, field_names=('x', 'y'), skip_nans=True)]
        clouds.append((ts, pts))
    elif topic == '/tf':
        for tf in deserialize_message(data, TFMessage).transforms:
            tfs.append((ts, tf))
    elif topic == '/odom':
        odoms.append((ts, deserialize_message(data, Odometry)))
    elif topic == '/local_costmap/costmap':
        maps.append((ts, deserialize_message(data, OccupancyGrid)))

print(f'读取: scan {len(scans)} cloud {len(clouds)} tf {len(tfs)} odom {len(odoms)} map {len(maps)}')

# ---- 简易 TF 查询：按 (parent, child) 存，查询时取时间最近的样本 ----
from collections import defaultdict
tf_by_key = defaultdict(list)
for ts, tf in tfs:
    tf_by_key[(tf.header.frame_id, tf.child_frame_id)].append((ts, tf))
for k in tf_by_key:
    tf_by_key[k].sort(key=lambda x: x[0])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def nearest_tf(parent, child, t):
    arr = tf_by_key.get((parent, child))
    if not arr:
        return None
    best = min(arr, key=lambda x: abs(x[0] - t))
    return best[1]


def compose_rot(tf, p):
    """p_child -> p_parent（只做 2D：绕 z 旋转 + 平移）"""
    yaw = yaw_of(tf.transform.rotation)
    c, s = math.cos(yaw), math.sin(yaw)
    tx, ty = tf.transform.translation.x, tf.transform.translation.y
    return (c * p[0] - s * p[1] + tx, s * p[0] + c * p[1] + ty)


# ===== 1) 去畸变到底移了多少点 =====
# 旋转检测：用 /odom 的角速度
odom_yaw_rate = []
for i in range(1, len(odoms)):
    dt = odoms[i][0] - odoms[i - 1][0]
    if dt <= 0:
        continue
    q0, q1 = odoms[i - 1][1].pose.pose.orientation, odoms[i][1].pose.pose.orientation
    dyaw = math.atan2(math.sin(yaw_of(q1) - yaw_of(q0)), math.cos(yaw_of(q1) - yaw_of(q0)))
    odom_yaw_rate.append((odoms[i][0], dyaw / dt))


def yaw_rate_at(t):
    if not odom_yaw_rate:
        return 0.0
    best = min(odom_yaw_rate, key=lambda x: abs(x[0] - t))
    return best[1]


# 每个 cloud 找最近的 scan，比较"云点角度 vs 原扫描该角度处的半径"，以及云点与原始点的位移
shift_stat = {'rot': [], 'still': []}
for tc, pts in clouds:
    if not pts:
        continue
    # 最近 scan
    if not scans:
        break
    ts_scan, scan = min(scans, key=lambda x: abs(x[0] - tc))
    w = abs(yaw_rate_at(tc))
    # 原始点：角度-半径
    n = len(scan.ranges)
    angs = [scan.angle_min + i * scan.angle_increment for i in range(n)]
    rngs = list(scan.ranges)
    shifts = []
    for (x, y) in pts:
        r_c = math.hypot(x, y)
        if r_c < scan.range_min:
            continue
        a_c = math.atan2(y, x)
        # 找原扫描里角度最接近的束的半径
        dangs = [abs(math.atan2(math.sin(a_c - a), math.cos(a_c - a))) for a in angs]
        i = int(np.argmin(dangs))
        r_raw = rngs[i]
        if math.isfinite(r_raw) and r_raw > scan.range_min:
            shifts.append(abs(r_c - r_raw))
    if shifts:
        key = 'rot' if w > 0.2 else 'still'
        shift_stat[key].extend(shifts)

for k in ('rot', 'still'):
    if shift_stat[k]:
        s = np.array(shift_stat[k])
        print(f'[去畸变点位移] {k}: 点数{len(s)} 均值{s.mean()*100:.2f}cm 最大{s.max()*100:.1f}cm '
              f'>3cm比例{(s>0.03).mean()*100:.0f}%')
    else:
        print(f'[去畸变点位移] {k}: 无数据')

# ===== 2) 代价地图致命格解释率（用去畸变点云）=====
if maps and clouds:
    tm, cmap = maps[-1]
    g = np.array(cmap.data, dtype=np.int16).reshape(cmap.info.height, cmap.info.width)
    lethal = g >= 99
    if lethal.sum() == 0:
        print('[代价地图] 最后一帧致命格为 0，无法分析')
    else:
        # 最近 cloud
        tc, pts = min(clouds, key=lambda x: abs(x[0] - tm))
        # cloud 是 laser 系，需 laser->odom（用静态 laser->base + odom->base）
        # 先找 laser 与 base 的父子
        tf_bl = None
        for k in tf_by_key:
            if 'laser' in k[0] or 'laser' in k[1]:
                if 'base' in k[0] or 'base' in k[1]:
                    tf_bl = nearest_tf(k[0], k[1], tc)
                    break
        tf_ob = nearest_tf('odom', 'base_footprint', tc)
        explained = 0
        total = 0
        if tf_bl is not None and tf_ob is not None:
            ox, oy, res = cmap.info.origin.position.x, cmap.info.origin.position.y, cmap.info.resolution
            # 激光点 -> odom
            odom_pts = []
            for p in pts:
                # 先 laser -> base
                if tf_bl.header.frame_id == 'base_footprint':
                    p_base = compose_rot(tf_bl, p)
                else:  # 反着，近似忽略（本车应一致）
                    p_base = p
                p_odom = compose_rot(tf_ob, p_base)
                odom_pts.append(p_odom)
            # 栅格化
            grid = set()
            for (x, y) in odom_pts:
                c = int((x - ox) / res)
                r = int((y - oy) / res)
                for drr in (-1, 0, 1):
                    for dcc in (-1, 0, 1):
                        grid.add((r + drr, c + dcc))
            for r in range(cmap.info.height):
                for c in range(cmap.info.width):
                    if lethal[r, c]:
                        total += 1
                        if (r, c) in grid:
                            explained += 1
            print(f'[代价地图解释] 致命格{total}，被去畸变点云解释 {explained} ({100*explained/max(1,total):.0f}%)')
        else:
            print(f'[代价地图解释] 缺 TF: laser->base={tf_bl is not None} odom->base={tf_ob is not None}')

# ===== 3) map->odom 逐帧抖动 =====
mo = [(ts, tf) for ts, tf in tf_by_key.get(('map', 'odom'), [])]
if len(mo) > 3:
    yaws = [yaw_of(tf.transform.rotation) for _, tf in mo]
    ts = [t for t, _ in mo]
    dts = np.diff(ts)
    dyaws = np.degrees([abs(math.atan2(math.sin(yaws[i + 1] - yaws[i]), math.cos(yaws[i + 1] - yaws[i]))) for i in range(len(yaws) - 1)])
    print(f'[map->odom] {len(mo)}帧 平均间隔{dts.mean()*1000:.0f}ms | '
          f'yaw每帧 均值{dyaws.mean():.3f}° 最大{dyaws.max():.2f}° | >0.2°帧数 {int((dyaws>0.2).sum())}/{len(dyaws)}')
else:
    print(f'[map->odom] TF 样本不足({len(mo)})')
