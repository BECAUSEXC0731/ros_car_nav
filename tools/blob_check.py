#!/usr/bin/python3
"""把局部代价地图的致命格变换到 map 系，与静态地图比对：
   - 落在真实墙线附近 → 真障碍
   - 落在离墙较远处   → 残留/拖影
"""
import math
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy import ndimage


def qos():
    return QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class N(rclpy.node.Node):
    def __init__(self):
        super().__init__('blob_check')
        self.lc = []
        self.gm = []
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: self.lc.append(m), qos())
        self.create_subscription(OccupancyGrid, '/map', lambda m: self.gm.append(m), qos())
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)


def main():
    rclpy.init()
    n = N()
    t0 = time.time()
    while time.time() - t0 < 6.0 and (not n.lc or not n.gm):
        rclpy.spin_once(n, timeout_sec=0.05)
    if not n.lc or not n.gm:
        print('数据不足'); return
    lc, gm = n.lc[-1], n.gm[-1]
    try:
        t = n.buf.lookup_transform('map', lc.header.frame_id, rclpy.time.Time()).transform
    except Exception as e:  # noqa: BLE001
        print('拿不到 map->', lc.header.frame_id, e); return
    q = t.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)

    w, h, res = lc.info.width, lc.info.height, lc.info.resolution
    g = np.array(lc.data, dtype=np.int16).reshape(h, w)
    ys, xs = np.nonzero(g >= 99)
    # 局部 costmap 栅格 -> odom -> map
    xo = xs * res + lc.info.origin.position.x
    yo = ys * res + lc.info.origin.position.y
    xm = c * xo - s * yo + t.translation.x
    ym = s * xo + c * yo + t.translation.y

    # 静态地图：每格到最近障碍的距离（米）
    gw, gh, gres = gm.info.width, gm.info.height, gm.info.resolution
    occ = np.array(gm.data, dtype=np.int16).reshape(gh, gw) >= 99
    dist = ndimage.distance_transform_edt(~occ) * gres
    cols = np.floor((xm - gm.info.origin.position.x) / gres).astype(int)
    rows = np.floor((ym - gm.info.origin.position.y) / gres).astype(int)
    inb = (cols >= 0) & (cols < gw) & (rows >= 0) & (rows < gh)
    d = np.full(len(xs), np.nan)
    d[inb] = dist[rows[inb], cols[inb]]

    print(f'局部代价地图致命格 {len(xs)} 个（{w}x{h} @ {res}，frame={lc.header.frame_id}）')
    print(f'map->odom: 平移 ({t.translation.x:.2f}, {t.translation.y:.2f}) yaw {math.degrees(yaw):.1f}°')
    for lo, hi in [(0, 0.10), (0.10, 0.25), (0.25, 0.50), (0.50, 1.0), (1.0, 99)]:
        cnt = int(np.count_nonzero((d >= lo) & (d < hi)))
        print(f'  距静态地图障碍 {lo*100:>3.0f}~{hi*100 if hi < 90 else 999:>3} cm : {cnt:5d} 格 '
              f'({100*cnt/max(1,len(xs)):4.1f}%)')
    out = int(np.count_nonzero(~inb))
    print(f'  落在地图外: {out} 格')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
