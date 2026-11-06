#!/usr/bin/python3
"""实测：/scan 与 /scan_ts 的频率、时间戳年龄，以及 /amcl_pose 抖动。"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped


class M(Node):
    def __init__(self):
        super().__init__('measure')
        self.s = []      # (recv_time, stamp, n_valid)
        self.ts = []
        self.p = []
        self.create_subscription(LaserScan, 'scan', self.cb_s, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_ts', self.cb_ts, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.cb_p, 10)

    @staticmethod
    def _valid(m):
        r = np.array(m.ranges, dtype=np.float32)
        return int(np.count_nonzero(np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)))

    def cb_s(self, m):
        self.s.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, self._valid(m)))

    def cb_ts(self, m):
        self.ts.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, self._valid(m)))

    def cb_p(self, m):
        p, q = m.pose.pose, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.p.append((time.time(), p.position.x, p.position.y, yaw))


def stats(tag, arr, dur):
    if len(arr) < 3:
        print(f'[{tag}] 只收到 {len(arr)} 帧')
        return
    t = np.array([a[0] for a in arr])
    stamp = np.array([a[1] for a in arr])
    age = t - stamp
    dt = np.diff(t)
    dt_st = np.diff(stamp)
    print(f'[{tag}] {len(arr)} 帧 / {dur:.1f}s = {len(arr)/dur:.1f} Hz | '
          f'平均间隔 {dt.mean()*1000:.0f}ms (最大 {dt.max()*1000:.0f}ms) | '
          f'消息内间隔 {dt_st.mean()*1000:.0f}ms | '
          f'时间戳年龄 均值 {age.mean()*1000:.0f}ms 最大 {age.max()*1000:.0f}ms | '
          f'有效点 {int(np.mean([a[2] for a in arr]))}')


def main():
    rclpy.init()
    n = M()
    dur = 8.0
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)
    stats('/scan', n.s, dur)
    stats('/scan_ts', n.ts, dur)
    if len(n.p) > 3:
        p = n.p
        d = [(math.hypot(p[i][1]-p[i-1][1], p[i][2]-p[i-1][2]),
              math.degrees(abs(math.atan2(math.sin(p[i][3]-p[i-1][3]),
                                          math.cos(p[i][3]-p[i-1][3])))))
             for i in range(1, len(p))]
        d = np.array(d)
        print(f'[/amcl_pose] {len(p)} 帧 = {len(p)/dur:.1f} Hz | 相邻帧位移 均值 {d[:,0].mean()*100:.1f}cm '
              f'最大 {d[:,0].max()*100:.1f}cm | 相邻帧转角 均值 {d[:,1].mean():.2f}° '
              f'最大 {d[:,1].max():.2f}° | 零位移帧 {int((d[:,0] < 1e-4).sum())}/{len(d)}')
    else:
        print('[/amcl_pose] 数据不足')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
