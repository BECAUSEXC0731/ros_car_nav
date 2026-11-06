#!/usr/bin/python3
"""把 /local_costmap/costmap 打成 ASCII 图，并统计连通块（找“一大块”是什么）。

图例： # = 致命障碍(>=99)   + = 内切/膨胀(1~98)   · = 自由(0)   空格 = 未知(-1)
"""
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy import ndimage

topic = sys.argv[1] if len(sys.argv) > 1 else '/local_costmap/costmap'

rclpy.init()
node = rclpy.create_node('dump_costmap')
qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
msgs = []
node.create_subscription(OccupancyGrid, topic, lambda m: msgs.append(m), qos)
tf_buf = tf2_ros.Buffer()
tf2_ros.TransformListener(tf_buf, node)

t0 = time.time()
while time.time() - t0 < 6.0 and not msgs:
    rclpy.spin_once(node, timeout_sec=0.1)
if not msgs:
    print('没收到', topic)
    raise SystemExit(0)
m = msgs[-1]
w, h, res = m.info.width, m.info.height, m.info.resolution
g = np.array(m.data, dtype=np.int16).reshape(h, w)

# 机器人位置（换算到该 costmap 的帧里）
rx = ry = None
try:
    ts = tf_buf.lookup_transform(m.header.frame_id, 'base_footprint', rclpy.time.Time())
    rx = (ts.transform.translation.x - m.info.origin.position.x) / res
    ry = (ts.transform.translation.y - m.info.origin.position.y) / res
except Exception as e:  # noqa: BLE001
    print('拿不到机器人位置:', e)

print(f'{topic}  {w}x{h} @ {res} m，origin=({m.info.origin.position.x:.2f},'
      f'{m.info.origin.position.y:.2f}) frame={m.header.frame_id}')
print(f'致命 {int((g>=99).sum())}  膨胀 {int(((g>0)&(g<99)).sum())}  未知 {int((g<0).sum())}')

# 从上往下打印（行索引大的在上，符合 RViz 视觉习惯）
print('   ' + ''.join('-' for _ in range(w)))
for row in range(h - 1, -1, -1):
    line = []
    for col in range(w):
        v = g[row, col]
        ch = '#' if v >= 99 else ('+' if v > 0 else ('·' if v == 0 else ' '))
        if rx is not None and abs(col - rx) < 1.2 and abs(row - ry) < 1.2:
            ch = 'R'
        line.append(ch)
    print(f'{row:3d}' + ''.join(line))

# 连通块
lethal = g >= 99
lab, n = ndimage.label(lethal, structure=np.ones((3, 3)))
if n:
    sizes = ndimage.sum(lethal, lab, range(1, n + 1))
    order = np.argsort(sizes)[::-1]
    print(f'\n致命格连通块 {n} 个，最大的 5 个：')
    for i in order[:5]:
        ys, xs = np.nonzero(lab == i + 1)
        cx = (xs.mean() + 0.5) * res + m.info.origin.position.x
        cy = (ys.mean() + 0.5) * res + m.info.origin.position.y
        ext_x = (xs.max() - xs.min() + 1) * res
        ext_y = (ys.max() - ys.min() + 1) * res
        rel = ''
        if rx is not None:
            rel = (f'  距机器人 ({(xs.mean()-rx)*res:+.2f}, {(ys.mean()-ry)*res:+.2f}) m, '
                   f'{np.hypot((xs.mean()-rx)*res, (ys.mean()-ry)*res):.2f} m')
        print(f'  块{i+1}: {int(sizes[i])} 格, 中心=({cx:.2f},{cy:.2f}), '
              f'外接 {ext_x:.2f}x{ext_y:.2f} m{rel}')
node.destroy_node()
rclpy.shutdown()
