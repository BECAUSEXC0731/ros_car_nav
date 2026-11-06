#!/usr/bin/python3
"""判断局部代价地图里的致命格是"真实障碍"还是"残留标记"：

1. 把 /scan_ts 的点按 TF 变换到 costmap 的坐标系（odom），栅格化；
2. 与 costmap 的致命格比较：有多少致命格附近有激光点（被解释），多少没有（疑似残留）；
3. 隔 8 s 再取一次 costmap，看这些"没被解释"的格子是否还在（残留 = 一直不动）。
"""
import math
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from scipy import ndimage


class Cmp(rclpy.node.Node):
    def __init__(self):
        super().__init__('cmp_costmap')
        self.maps = []
        self.scan = None
        q = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.cb_map, q)
        self.create_subscription(LaserScan, '/scan_ts', self.cb_scan, qos_profile_sensor_data)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

    def cb_map(self, m):
        self.maps.append(m)

    def cb_scan(self, m):
        self.scan = m


def raster(scan, buf, frame):
    """把激光点变到 frame 后栅格化（返回每点对应坐标，米）"""
    ts = buf.lookup_transform(frame, scan.header.frame_id, rclpy.time.Time())
    t = ts.transform.translation
    q = ts.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array(scan.ranges, dtype=np.float32)
    a = scan.angle_min + np.arange(r.size) * scan.angle_increment
    ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    r, a = r[ok], a[ok]
    x = r * np.cos(a)
    y = r * np.sin(a)
    return c * x - s * y + t.x, s * x + c * y + t.y


def main():
    rclpy.init()
    n = Cmp()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not n.maps or n.scan is None:
        print('数据不足'); return
    m1 = n.maps[-1]
    w, h, res = m1.info.width, m1.info.height, m1.info.resolution
    ox, oy = m1.info.origin.position.x, m1.info.origin.position.y
    g = np.array(m1.data, dtype=np.int16).reshape(h, w)
    lethal = g >= 99

    px, py = raster(n.scan, n.buf, m1.header.frame_id)
    pts = np.zeros_like(lethal, dtype=bool)
    cols = np.floor((px - ox) / res).astype(int)
    rows = np.floor((py - oy) / res).astype(int)
    ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    pts[rows[ok], cols[ok]] = True
    # 膨胀 1 格，容忍 5cm 栅格化误差
    pts_d = ndimage.binary_dilation(pts, np.ones((3, 3)))

    explained = int((lethal & pts_d).sum())
    unexplained = int((lethal & ~pts_d).sum())
    print(f'代价地图致命格 {int(lethal.sum())} | 当前激光点能解释 {explained} '
          f'({100*explained/max(1,lethal.sum()):.0f}%) | 无法解释 {unexplained}')
    if unexplained:
        lab, ncc = ndimage.label(lethal & ~pts_d, structure=np.ones((3, 3)))
        sizes = ndimage.sum(lethal & ~pts_d, lab, range(1, ncc + 1))
        order = np.argsort(sizes)[::-1][:3]
        for i in order:
            ys, xs = np.nonzero(lab == i + 1)
            print(f'  残留块: {int(sizes[i])} 格, 中心=({(xs.mean()+0.5)*res+ox:.2f},'
                  f'{(ys.mean()+0.5)*res+oy:.2f}), 外接 '
                  f'{(xs.max()-xs.min()+1)*res:.2f}x{(ys.max()-ys.min()+1)*res:.2f} m')

    # 8 秒后再看一次：残留块是否一动不动
    t1 = time.time()
    while time.time() - t1 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    m2 = n.maps[-1]
    g2 = np.array(m2.data, dtype=np.int16).reshape(
        m2.info.height, m2.info.width)
    l2 = g2 >= 99
    # 两张图尺寸/原点若不同，用平移对齐
    dc = int(round((m2.info.origin.position.x - ox) / res))
    dr = int(round((m2.info.origin.position.y - oy) / res))
    same = 0
    total = 0
    ys, xs = np.nonzero(lethal & ~pts_d)
    for r_, c_ in zip(ys, xs):
        r2, c2 = r_ - dr, c_ - dc
        if 0 <= r2 < l2.shape[0] and 0 <= c2 < l2.shape[1]:
            total += 1
            same += int(l2[r2, c2])
    if total:
        print(f'8 s 后：这些"未解释"格里仍有 {same}/{total} ({100*same/total:.0f}%) 还是致命 '
              f'→ 占比高说明是没被清除的残留标记')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
