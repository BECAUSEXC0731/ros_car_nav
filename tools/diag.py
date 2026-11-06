#!/usr/bin/python3
"""实车诊断：/scan 数据质量 + /amcl_pose 逐帧抖动。"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped


class Diag(Node):
    def __init__(self):
        super().__init__('diag')
        self.scan_n = 0
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_ts', self.on_scan_ts, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, 10)
        self.last_pose = None
        self.deltas = []
        self.pose_n = 0

    def analyze(self, tag, msg):
        r = np.array(msg.ranges, dtype=np.float32)
        n = r.size
        finite = np.isfinite(r)
        inrange = finite & (r >= msg.range_min) & (r <= msg.range_max)
        zeros = np.count_nonzero(finite & (r <= 0.0))
        # 孤立点：与左右邻居都相差超过 0.3m
        isolated = 0
        for i in range(1, n - 1):
            if inrange[i] and inrange[i - 1] and inrange[i + 1]:
                if abs(r[i] - r[i - 1]) > 0.3 and abs(r[i] - r[i + 1]) > 0.3:
                    isolated += 1
        print(f'[{tag}] 共{n}束 有效{np.count_nonzero(inrange)} 非有限{np.count_nonzero(~finite)} '
              f'越界{np.count_nonzero(finite & ~inrange)} 零/负{zeros} '
              f'孤立噪点{isolated} | angle[{math.degrees(msg.angle_min):.0f},'
              f'{math.degrees(msg.angle_max):.0f}]° inc={math.degrees(msg.angle_increment):.2f}° '
              f'range=[{msg.range_min:.2f},{msg.range_max:.2f}] frame={msg.header.frame_id}', flush=True)

    def on_scan(self, msg):
        self.scan_n += 1
        if self.scan_n <= 3:
            self.analyze('scan', msg)

    def on_scan_ts(self, msg):
        if self.scan_n <= 3:
            self.analyze('scan_ts', msg)

    def on_pose(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        cur = (p.position.x, p.position.y, yaw)
        if self.last_pose is not None:
            d = math.hypot(cur[0] - self.last_pose[0], cur[1] - self.last_pose[1])
            dyaw = math.degrees(abs(math.atan2(math.sin(cur[2] - self.last_pose[2]),
                                               math.cos(cur[2] - self.last_pose[2]))))
            self.deltas.append((d, dyaw))
        self.last_pose = cur
        self.pose_n += 1

    def report(self):
        if not self.deltas:
            print('没收到 amcl_pose', flush=True)
            return
        d = np.array(self.deltas)
        print(f'[amcl_pose] {self.pose_n} 帧 | 相邻帧位移: 均值 {d[:,0].mean()*100:.1f}cm '
              f'最大 {d[:,0].max()*100:.1f}cm | 相邻帧转角: 均值 {d[:,1].mean():.2f}° '
              f'最大 {d[:,1].max():.2f}° | 零位移帧 {np.count_nonzero(d[:,0] < 1e-4)}', flush=True)


def main():
    rclpy.init()
    node = Diag()
    import time
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
