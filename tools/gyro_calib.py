#!/usr/bin/python3
"""陀螺标度因子测量：原地转一整圈，对比陀螺积分角度 vs 真实 360°。

用法：/usr/bin/python3 tools/gyro_calib.py [时长秒，默认20]
流程：
  1) 前 2 秒静止 —— 测陀螺零偏（你别动车）
  2) 然后原地转【正好 360°】（顺时针或逆时针都行，转满一整圈）
  3) 转完停稳别动，等脚本自动结束
结束时打印：
  - 陀螺积分角度（已扣零偏）
  - 轮子里程计角度变化（对照）
  - 若按真实 360° 反推的标度因子
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

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

rclpy.init()
n = rclpy.create_node('gyro_calib')
gyro = []      # (t, wz) 原始 /imu 角速度
odom_yaw = []  # (t, yaw) 轮子里程计朝向

n.create_subscription(
    Imu, 'imu',
    lambda m: gyro.append((time.time(), m.angular_velocity.z)),
    qos_profile_sensor_data)


def cb_odom(m):
    q = m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    odom_yaw.append((time.time(), yaw))


n.create_subscription(Odometry, 'odom', cb_odom, qos_profile_sensor_data)

t0 = time.time()
print('前 2 秒测零偏，别动车...', flush=True)
while time.time() - t0 < 2.0:
    rclpy.spin_once(n, timeout_sec=0.05)

bias_samples = [w for t, w in gyro if t - t0 < 2.0]
bias = float(np.mean(bias_samples)) if bias_samples else 0.0
print(f'零偏 ≈ {bias * 180 / math.pi:.3f}°/s', flush=True)
print(f'现在请原地转【正好 360°】(一整圈)，转完停稳别动，'
      f'{dur:.0f} 秒后脚本自动结束...', flush=True)

while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)

# 积分角速度（扣零偏）
gyro.sort(key=lambda x: x[0])
angle = 0.0
for i in range(1, len(gyro)):
    dt = gyro[i][0] - gyro[i - 1][0]
    if dt <= 0 or dt > 0.2:  # 跳过长时间断档
        continue
    angle += (gyro[i][1] - bias) * dt
gyro_deg = angle * 180.0 / math.pi

# 轮子里程计 yaw 变化（对照）
if len(odom_yaw) > 1:
    dyaw = odom_yaw[-1][1] - odom_yaw[0][1]
    odom_deg = math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw)))
else:
    odom_deg = float('nan')

print('\n===== 结果 =====', flush=True)
print(f'陀螺积分角度(扣零偏): {gyro_deg:.1f}°', flush=True)
print(f'轮子里程计角度变化:    {odom_deg:.1f}°', flush=True)
print(f'零偏:                  {bias * 180 / math.pi:.3f}°/s', flush=True)
if abs(gyro_deg) > 10:
    k = 360.0 / abs(gyro_deg)
    print(f'\n如果真实转了 360°，标度因子 k = 360 / {abs(gyro_deg):.1f} = {k:.4f}',
          flush=True)
    print('（陀螺读数偏小 → k>1；偏大 → k<1）', flush=True)
    print(f'\n请把上面 3 个数字发我：陀螺角度 / 里程计角度 / 标度因子', flush=True)

n.destroy_node()
rclpy.shutdown()
