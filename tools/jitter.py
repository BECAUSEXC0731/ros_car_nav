#!/usr/bin/python3
"""测稳态抖动：记录 /amcl_pose 一段时间，跳过前 skip 秒的收敛过程再统计。"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    total = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0
    skip = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
    rclpy.init()
    node = rclpy.create_node('jitter')
    poses = []

    def cb(msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        poses.append((time.time(), p.position.x, p.position.y, yaw))

    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', cb, 10)
    t0 = time.time()
    while time.time() - t0 < total:
        rclpy.spin_once(node, timeout_sec=0.05)

    arr = [p for p in poses if p[0] - t0 >= skip]
    if len(arr) < 5:
        print('[jitter] 数据太少', len(arr), flush=True)
        return
    d = np.array([(arr[i][1] - arr[i - 1][1], arr[i][2] - arr[i - 1][2],
                   math.degrees(abs(math.atan2(math.sin(arr[i][3] - arr[i - 1][3]),
                                               math.cos(arr[i][3] - arr[i - 1][3])))))
                  for i in range(1, len(arr))])
    trans = np.hypot(d[:, 0], d[:, 1])
    print('[jitter] 稳态 %d 帧, %.1f Hz | 位移 均值 %.1fcm 最大 %.1fcm | '
          '转角 均值 %.3f° 最大 %.3f° | 位姿取值个数 x=%d y=%d yaw=%d'
          % (len(arr), len(arr) / (total - skip), trans.mean() * 100, trans.max() * 100,
             d[:, 2].mean(), d[:, 2].max(),
             len(set(round(p[1], 4) for p in arr)),
             len(set(round(p[2], 4) for p in arr)),
             len(set(round(p[3], 5) for p in arr))), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
