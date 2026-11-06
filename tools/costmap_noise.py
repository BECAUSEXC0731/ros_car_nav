#!/usr/bin/python3
"""检查局部代价地图里的“孤立噪点”：统计被标记的格子以及孤立的单格。"""
import sys

import numpy as np
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

topic = sys.argv[1] if len(sys.argv) > 1 else '/local_costmap/costmap'

rclpy.init()
node = rclpy.create_node('costmap_noise')
qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                 reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
got = []


def cb(msg):
    got.append(msg)


node.create_subscription(OccupancyGrid, topic, cb, qos)
import time
t0 = time.time()
while time.time() - t0 < 6.0 and not got:
    rclpy.spin_once(node, timeout_sec=0.1)
if not got:
    print(f'[{topic}] 没收到数据')
    raise SystemExit(0)

m = got[-1]
w, h = m.info.width, m.info.height
g = np.array(m.data, dtype=np.int16).reshape(h, w)
lethal = g >= 99
inscribed = (g >= 1) & (g < 99)
# 孤立标记：该格被标记，但 8 邻域里没有其它被标记格
res = np.zeros_like(lethal, dtype=np.int16)
res[1:-1, 1:-1] = (
    lethal[:-2, :-2] + lethal[:-2, 1:-1] + lethal[:-2, 2:] +
    lethal[1:-1, :-2] + lethal[1:-1, 2:] +
    lethal[2:, :-2] + lethal[2:, 1:-1] + lethal[2:, 2:])
isolated = lethal & (res == 0)
print(f'[{topic}] {w}x{h} @ {m.info.resolution:.2f}m | 致命格 {int(lethal.sum())} '
      f'膨胀格 {int(inscribed.sum())} | 孤立致命格 {int(isolated.sum())}')
if isolated.any():
    ys, xs = np.nonzero(isolated)
    # 换算到地图坐标系（近似局部坐标）
    pts = [((x + 0.5) * m.info.resolution + m.info.origin.position.x,
            (y + 0.5) * m.info.resolution + m.info.origin.position.y) for x, y in zip(xs[:8], ys[:8])]
    print('  孤立点示例(米):', [(round(a, 2), round(b, 2)) for a, b in pts])
node.destroy_node()
rclpy.shutdown()
