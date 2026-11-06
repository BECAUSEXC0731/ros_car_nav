#!/usr/bin/python3
"""360° 旋转对比：陀螺 / 轮子里程计 / EKF 各转了多少。

用法：/usr/bin/python3 tools/yaw_compare.py [时长秒，默认20]
流程：
  1) 前 2 秒静止测零偏（别动车）
  2) 原地转【正好 360°】，转完停稳别动
结束时打印三行，看 EKF yaw 更接近 360°（跟陀螺）还是接近轮子的坏值：
  - 陀螺角度（已按 1.02 标定）
  - 轮子里程计角度
  - EKF 的 odom->base yaw 变化
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
GYRO_SCALE = 1.02  # 与 imu_repub.py 的 gyro_scale 保持一致

rclpy.init()
n = rclpy.create_node('yaw_compare')
gyro = []
odom_yaw = []
ekf_yaw = []

n.create_subscription(
    Imu, 'imu',
    lambda m: gyro.append((time.time(), m.angular_velocity.z)),
    qos_profile_sensor_data)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def cb_odom(m):
    odom_yaw.append((time.time(), yaw_of(m.pose.pose.orientation)))


def cb_tf(m):
    for tf in m.transforms:
        if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
            ekf_yaw.append((time.time(), yaw_of(tf.transform.rotation)))


n.create_subscription(Odometry, 'odom', cb_odom, qos_profile_sensor_data)
n.create_subscription(TFMessage, 'tf', cb_tf, 100)

t0 = time.time()
print('前 2 秒测零偏，别动车...', flush=True)
while time.time() - t0 < 2.0:
    rclpy.spin_once(n, timeout_sec=0.05)

bias = float(np.mean([w for t, w in gyro if t - t0 < 2.0])) if gyro else 0.0
print(f'零偏 ≈ {bias * 180 / math.pi:.3f}°/s', flush=True)
print(f'现在请原地转【正好 360°】(一整圈)，转完停稳别动，'
      f'{dur:.0f} 秒后脚本自动结束...', flush=True)

while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)

# 陀螺积分（扣零偏 + 乘标定系数）
gyro.sort(key=lambda x: x[0])
angle = 0.0
for i in range(1, len(gyro)):
    dt = gyro[i][0] - gyro[i - 1][0]
    if dt <= 0 or dt > 0.2:
        continue
    angle += (gyro[i][1] - bias) * dt
gyro_deg = angle * 180.0 / math.pi * GYRO_SCALE


def delta_yaw(arr):
    """把带 ±180° 跳变的 yaw 序列解卷成连续角度，返回累计变化（一整圈=360°，不会被折成0）。"""
    if len(arr) < 2:
        return float('nan')
    arr.sort(key=lambda x: x[0])
    total = 0.0
    prev = arr[0][1]
    for i in range(1, len(arr)):
        cur = arr[i][1]
        d = math.atan2(math.sin(cur - prev), math.cos(cur - prev))
        total += d
        prev = cur
    return math.degrees(total)


odom_deg = delta_yaw(odom_yaw)
ekf_deg = delta_yaw(ekf_yaw)

print('\n===== 结果 =====', flush=True)
print(f'陀螺角度(标定后):  {gyro_deg:.1f}°', flush=True)
print(f'轮子里程计角度:    {odom_deg:.1f}°', flush=True)
print(f'EKF yaw 变化:      {ekf_deg:.1f}°', flush=True)
print('\n判断：EKF yaw 接近 360° = 跟陀螺（轮子没污染 EKF，去查 TF 时间戳）；'
      '接近 162° = 轮子污染了 EKF', flush=True)

n.destroy_node()
rclpy.shutdown()
