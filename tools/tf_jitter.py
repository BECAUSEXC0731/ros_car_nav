#!/usr/bin/python3
"""测量 map->odom（lidar_loc）与 odom->base_footprint（EKF）两条 TF 的逐帧抖动。

用法：/usr/bin/python3 tools/tf_jitter.py <时长s> [跳过前Ns]
旋转/移动小车时跑，看哪条 TF 在跳，定位"RViz 里小车抖"的来源。
"""
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.time import Time

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
skip = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

rclpy.init()
n = rclpy.create_node('tf_jitter')
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, n)

mo = []   # map->odom  (t, x, y, yaw)
ob = []   # odom->base (t, x, y, yaw)

def yaw_of(tf):
    q = tf.transform.rotation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.02)
    now = time.time()
    if now - t0 < skip:
        continue
    try:
        ts = buf.lookup_transform('map', 'odom', Time())
        mo.append((now, ts.transform.translation.x, ts.transform.translation.y, yaw_of(ts)))
    except Exception:
        pass
    try:
        ts = buf.lookup_transform('odom', 'base_footprint', Time())
        ob.append((now, ts.transform.translation.x, ts.transform.translation.y, yaw_of(ts)))
    except Exception:
        pass

def report(tag, arr):
    if len(arr) < 3:
        print(f'[{tag}] 数据不足({len(arr)}帧)')
        return
    a = np.array(arr)
    dt = np.diff(a[:, 0])
    dxy = np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2]))
    dyaw = np.degrees(np.abs(np.diff(a[:, 3])))
    # 把 >180° 的角度差折回 [0,180]
    dyaw = np.where(dyaw > 180, 360 - dyaw, dyaw)
    print(f'[{tag}] {len(a)}帧/{dur:.0f}s | 采样间隔 {dt.mean()*1000:.0f}ms | '
          f'位移每帧 均值{dxy.mean()*100:.1f}cm 最大{dxy.max()*100:.1f}cm | '
          f'转角每帧 均值{dyaw.mean():.2f}° 最大{dyaw.max():.2f}° | '
          f'转角>0.2°帧数 {int((dyaw>0.2).sum())}/{len(dyaw)}')

report('map->odom   (lidar_loc)', mo)
report('odom->base  (EKF)      ', ob)

n.destroy_node()
rclpy.shutdown()
