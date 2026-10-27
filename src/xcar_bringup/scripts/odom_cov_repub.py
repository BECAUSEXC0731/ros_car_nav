#!/usr/bin/python3
"""
odom_cov_repub.py —— 给下位机的 /odom 补一套"可信度"协方差，再转发给 EKF

要解决的问题（原地旋转打滑）：
  轮式里程计在"原地旋转"时轮子打滑，编码器推算的 yaw 会累积偏差；
  平移（x/y）基本不受影响。
  但 EKF 的 odom0_config 把 odom 的 yaw 当"绝对航向参照"，而下位机
  给的协方差往往是全 0（等于"无限信任"）——于是旋转时航向被一路拖偏。
  纯旋转时 2D 激光匹配对 yaw 又退化、纠不回来，表现就是：
  "原地转几圈后车头方向和雷达数据对不上，一前进才纠正回来"。

做法：
  复制 /odom 的位姿，只改协方差，发布成 /odom_cov：
      x / y  : 小方差 → 信平移（前进能纠回来靠的就是它）
      yaw    : 大方差 → 不信打滑的编码器航向，把旋转交给 IMU 的 vyaw
  EKF 的 odom0 改指 /odom_cov 即可，原始 /odom 不受影响，其它节点照常用。

注意：
  1. 只改协方差，header（含时间戳）原样保留 —— 避免引入时间戳偏移，
     否则 slam/AMCL 会报 TF 外推错误。
  2. 用 /usr/bin/python3 而不是 /usr/bin/env python3：ROS2 Humble 的 rclpy
     只对系统 Python 3.10 有效（与 imu_repub.py 同样的原因）。

参数：
  input_topic / output_topic                       输入输出话题（默认 odom / odom_cov）
  x_variance / y_variance                          平移协方差 [m^2]
  yaw_variance                                     yaw 协方差 [rad^2]，越大越不信编码器航向
  ignored_variance                                 z/roll/pitch 的兜底值（two_d_mode 下被忽略）
  twist_fallback_variance                          速度协方差全 0 时的兜底值
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

# 6x6 协方差数组（行优先）里的对角索引：x, y, z, roll, pitch, yaw
POSE_X, POSE_Y, POSE_Z = 0, 7, 14
POSE_ROLL, POSE_PITCH, POSE_YAW = 21, 28, 35

# 速度协方差同样是 6x6：vx, vy, vz, vroll, vpitch, vyaw
TWIST_DIAG = (0, 7, 14, 21, 28, 35)


class OdomCovRepub(Node):
    def __init__(self):
        super().__init__('odom_cov_repub')

        self.declare_parameter('input_topic', 'odom')
        self.declare_parameter('output_topic', 'odom_cov')
        # 平移：小方差 = 高度信任
        self.declare_parameter('x_variance', 5.0e-4)   # m^2
        self.declare_parameter('y_variance', 5.0e-4)   # m^2
        # 航向：大方差 = 不信任。5.0 rad^2 ≈ (77°)^2，
        # 旋转时 IMU 的 vyaw 几乎完全主导；直行/静止时 odom yaw 仍会
        # 缓慢地把 IMU 的积分漂移拉回来（时间常数几分钟量级）。
        # 调大 → 更不信任编码器航向（旋转更稳，但长期漂移纠得更慢）
        # 调小 → 更多信任编码器航向（静态更稳，但旋转时容易被拖偏）
        self.declare_parameter('yaw_variance', 5.0)    # rad^2
        # z/roll/pitch：two_d_mode 下 EKF 不看，填非 0 只为让协方差矩阵可逆
        self.declare_parameter('ignored_variance', 1.0)
        # 速度项兜底（EKF 当前 odom0_config 未使用速度）
        self.declare_parameter('twist_fallback_variance', 1.0e-2)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.x_var_ = self.get_parameter('x_variance').value
        self.y_var_ = self.get_parameter('y_variance').value
        self.yaw_var_ = self.get_parameter('yaw_variance').value
        self.ignored_var_ = self.get_parameter('ignored_variance').value
        self.twist_var_ = self.get_parameter('twist_fallback_variance').value

        self.logged_input_cov_ = False

        self.get_logger().info(
            f'odom_cov_repub: {in_topic} -> {out_topic}, '
            f'x_var={self.x_var_:.1e}, y_var={self.y_var_:.1e}, '
            f'yaw_var={self.yaw_var_:.2f} rad^2 (越大越不信编码器航向)')

        # 用 sensor_data QoS，与下位机发布 /odom 的 QoS 匹配
        self.pub = self.create_publisher(Odometry, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Odometry, in_topic, self.callback, qos_profile_sensor_data)

    @staticmethod
    def _is_all_zero(cov):
        return all(v == 0.0 for v in cov)

    def callback(self, msg: Odometry):
        # 第一次收到消息时打印下位机原始协方差，便于确认是否全 0
        if not self.logged_input_cov_:
            self.logged_input_cov_ = True
            cov = msg.pose.covariance
            self.get_logger().info(
                '收到下位机 /odom，原始 pose 协方差对角 '
                f'[x={cov[POSE_X]:.3g}, y={cov[POSE_Y]:.3g}, yaw={cov[POSE_YAW]:.3g}]'
                f'{"  <- 全 0，EKF 原本等于无限信任它" if self._is_all_zero(cov) else ""}')

        # --- 位姿协方差：信平移、不信 yaw ---
        msg.pose.covariance[POSE_X] = self.x_var_
        msg.pose.covariance[POSE_Y] = self.y_var_
        msg.pose.covariance[POSE_YAW] = self.yaw_var_
        # two_d_mode 下用不到，但填上可保证协方差矩阵可逆（避免马氏距离算出 NaN）
        msg.pose.covariance[POSE_Z] = self.ignored_var_
        msg.pose.covariance[POSE_ROLL] = self.ignored_var_
        msg.pose.covariance[POSE_PITCH] = self.ignored_var_

        # --- 速度协方差：全 0 时兜底 ---
        if self._is_all_zero(msg.twist.covariance):
            for i in TWIST_DIAG:
                msg.twist.covariance[i] = self.twist_var_

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovRepub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # rclpy 在 Ctrl+C 竞争时可能抛 RuntimeError
        node.get_logger().warn(f'odom_cov_repub 退出: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
