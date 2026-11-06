#!/usr/bin/python3
"""搜索激光数据的"真实年龄"：用静态地图做基准，找哪个时间延迟对应的位姿能把
激光点对齐到地图上。

原理：/scan_ts 被 scan_repub 重打戳成"接收时刻"。如果数据内容其实比接收时刻旧 Δ，
那就该用 (stamp - Δ) 时刻的 TF 去变换它，对齐误差才最小。
机器人旋转时，Δ=0 与 Δ=0.47s 的位姿差可达十几度 → 误差差异极大，一眼能看出来。
"""
import math
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import LaserScan


class Age(rclpy.node.Node):
    def __init__(self):
        super().__init__('scan_age')
        self.map = None
        self.scans = []
        q = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, q)
        self.create_subscription(LaserScan, '/scan_ts', self.cb_scan, qos_profile_sensor_data)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

    def cb_map(self, m):
        self.map = m

    def cb_scan(self, m):
        if len(self.scans) < 12:
            self.scans.append(m)


def main():
    rclpy.init()
    n = Age()
    t0 = time.time()
    while time.time() - t0 < 6.0 and (n.map is None or len(n.scans) < 8):
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.map is None or not n.scans:
        print('数据不足'); return

    m = n.map
    w, hh, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    occ = np.array(m.data, dtype=np.int16).reshape(hh, w) >= 99
    # 地图每格到最近障碍的距离（米）
    dist = ndimage.distance_transform_edt(~occ) * res
    print(f'地图 {w}x{hh} @ {res}，占据格 {int(occ.sum())}')

    delays = [0.0, 0.15, 0.30, 0.45, 0.60, 0.75]
    errs = {d: [] for d in delays}
    yaws = []
    for s in n.scans[-8:]:
        stamp = rclpy.time.Time.from_msg(s.header.stamp)
        r = np.array(s.ranges, dtype=np.float32)
        a = s.angle_min + np.arange(r.size) * s.angle_increment
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        px, py = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        for d in delays:
            try:
                ts = n.buf.lookup_transform('map', s.header.frame_id,
                                            stamp - rclpy.duration.Duration(seconds=d))
            except Exception as e:  # noqa: BLE001
                if d == 0.0:
                    print('TF 失败:', type(e).__name__, e)
                continue
            q = ts.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            if d == 0.0:
                yaws.append(yaw)
            c, si = math.cos(yaw), math.sin(yaw)
            xm = c * px - si * py + ts.transform.translation.x
            ym = si * px + c * py + ts.transform.translation.y
            col = np.floor((xm - ox) / res).astype(int)
            row = np.floor((ym - oy) / res).astype(int)
            inb = (col >= 0) & (col < w) & (row >= 0) & (row < hh)
            if not inb.any():
                continue
            errs[d].append(np.median(dist[row[inb], col[inb]]))

    print(f'\n用不同延迟的位姿对齐到地图的误差（中位数，越小越对）：')
    for d in delays:
        if errs[d]:
            print(f'  delay {d*100:5.0f} ms → 误差 {np.median(errs[d])*100:5.1f} cm '
                  f'（{len(errs[d])} 帧）')
    if len(yaws) > 1:
        dyaw = np.degrees(np.abs(np.diff(yaws)))
        print(f'\n扫描期间机器人角速度约 {dyaw.mean()/0.143:.2f} °/s（0.143s 一帧）')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
