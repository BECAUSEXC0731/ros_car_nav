#!/usr/bin/python3
"""等待并打印 map->odom（最多 15 s），用于验证 lidar_loc 的 TF 输出。"""
import math
import sys
import time

import rclpy
import tf2_ros


def main():
    rclpy.init()
    node = rclpy.create_node('check_tf')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    got = 0
    while time.time() - t0 < 15.0 and got < 3:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            ts = buf.lookup_transform('map', 'odom', rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            continue
        got += 1
        q = ts.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        print('[check_tf] map->odom x=%.3f y=%.3f yaw=%.1f° (stamp %d.%09d)'
              % (ts.transform.translation.x, ts.transform.translation.y,
                 math.degrees(yaw), ts.header.stamp.sec, ts.header.stamp.nanosec),
              flush=True)
        time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()
    if got == 0:
        print('[check_tf] 没收到 map->odom', flush=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
