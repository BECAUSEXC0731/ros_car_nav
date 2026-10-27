#!/usr/bin/python3
"""
imu_repub.py —— 把下位机的 /imu 转成 EKF 能正确使用的 /imu_rotated

注意：这里用 /usr/bin/python3 而不是 /usr/bin/env python3。
ROS2 Humble 的 rclpy 只对系统 Python 3.10 有效，如果环境里装了 conda / pyenv，
`env python3` 可能解析到别的版本（如 3.14），节点会直接 ModuleNotFoundError。

做三件事：
  1. 把 header.frame_id 改成 imu_link
     这样 robot_localization 会通过 TF 树上 imu_link 的旋转，
     自动补偿 MPU 的实际安装朝向（见 xcar_description/urdf/xcar.urdf）
  2. 协方差兜底
     协方差全 0 会让 EKF 无法正确加权（甚至导致融合失效），
     所以只在"全 0"时填一组默认值；如果下位机已经给了真实协方差，就原样保留。
  3. 陀螺零偏自动估计与扣除（2026-10 新增）
     EKF 的航向现在主要靠这个 gyro 的 vyaw 积分（见 ekf.yaml 与
     odom_cov_repub.py 的说明），所以陀螺零偏会直接变成"原地旋转时
     慢慢偏移"。这里在"已指令停止 且 测到的 vyaw 足够小"时用滑动平均
     估计零偏，发布前扣掉；估计值会周期性打进日志，便于确认。

参数：
  frame_id               目标 frame（默认 imu_link）
  fill_zero_covariance   是否在协方差全 0 时填默认值（默认 true）
  orientation_variance / angular_velocity_variance  默认协方差数值
  estimate_gyro_bias     是否估计并扣除零偏（默认 true）
  cmd_vel_topic          用于判断"是否被指令移动"的话题（默认 cmd_vel）
  stationary_* / bias_*  零偏估计的阈值与时间常数
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class ImuRepub(Node):
    def __init__(self):
        super().__init__('imu_repub')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('fill_zero_covariance', True)
        self.declare_parameter('orientation_variance', 0.001)      # rad^2
        self.declare_parameter('angular_velocity_variance', 0.01)  # (rad/s)^2

        # --- 陀螺零偏自动估计与扣除 ---
        self.declare_parameter('estimate_gyro_bias', True)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        # |cmd_vel| 小于它才算"被指令停止"
        self.declare_parameter('stationary_velocity_threshold', 1.0e-3)
        # |vyaw| 小于它才参与零偏统计 [rad/s]（遥控最慢的原地转也远大于它）
        self.declare_parameter('stationary_gyro_threshold', 0.05)
        self.declare_parameter('stationary_delay', 1.0)         # 有 cmd_vel 时需连续静止 [s]
        self.declare_parameter('stationary_delay_no_cmd', 3.0)  # 收不到 cmd_vel 时更严 [s]
        self.declare_parameter('bias_time_constant', 2.0)       # 零偏滑动平均时间常数 [s]
        self.declare_parameter('bias_report_period', 5.0)       # 零偏日志周期 [s]

        self.frame_id_ = self.get_parameter('frame_id').value
        self.fill_zero_ = self.get_parameter('fill_zero_covariance').value
        self.ori_var_ = self.get_parameter('orientation_variance').value
        self.ang_var_ = self.get_parameter('angular_velocity_variance').value

        self.estimate_bias_ = self.get_parameter('estimate_gyro_bias').value
        self.stationary_vel_thr_ = \
            self.get_parameter('stationary_velocity_threshold').value
        self.stationary_gyro_thr_ = \
            self.get_parameter('stationary_gyro_threshold').value
        self.stationary_delay_ = self.get_parameter('stationary_delay').value
        self.stationary_delay_no_cmd_ = \
            self.get_parameter('stationary_delay_no_cmd').value
        self.bias_tc_ = self.get_parameter('bias_time_constant').value
        self.bias_report_period_ = self.get_parameter('bias_report_period').value

        # 零偏估计的运行状态
        self.bias_ = 0.0
        self.bias_ready_ = False
        self.stationary_time_ = 0.0
        self.last_stamp_ = None
        self.cmd_vel_stamp_ = None
        self.cmd_vel_linear_ = 0.0
        self.cmd_vel_angular_ = 0.0

        self.get_logger().info(
            f'imu_repub: /imu -> /imu_rotated, frame_id = {self.frame_id_}, '
            f'fill_zero_covariance = {self.fill_zero_}, '
            f'estimate_gyro_bias = {self.estimate_bias_}')

        # 使用 sensor_data QoS (BEST_EFFORT)，与下位机发布 IMU 的 QoS 匹配
        self.pub = self.create_publisher(Imu, 'imu_rotated', qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Imu, 'imu', self.callback, qos_profile_sensor_data)

        # 用 /cmd_vel 判断"是否被指令移动"。
        # BEST_EFFORT 订阅对 reliable / best_effort 两种发布端都兼容
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_sub_ = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, qos_profile_sensor_data)

        if self.estimate_bias_ and self.bias_report_period_ > 0.0:
            self.create_timer(self.bias_report_period_, self._report_bias)

    @staticmethod
    def _is_all_zero(cov):
        return all(v == 0.0 for v in cov)

    def callback(self, msg: Imu):
        msg.header.frame_id = self.frame_id_

        if self.estimate_bias_:
            # 用原始值估计零偏，再把零偏扣掉
            self._update_gyro_bias(msg.angular_velocity.z)
            msg.angular_velocity.z -= self.bias_

        if self.fill_zero_:
            if self._is_all_zero(msg.orientation_covariance):
                msg.orientation_covariance[0] = self.ori_var_
                msg.orientation_covariance[4] = self.ori_var_
                msg.orientation_covariance[8] = self.ori_var_
            if self._is_all_zero(msg.angular_velocity_covariance):
                msg.angular_velocity_covariance[0] = self.ang_var_
                msg.angular_velocity_covariance[4] = self.ang_var_
                msg.angular_velocity_covariance[8] = self.ang_var_

        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # 陀螺零偏估计
    # ------------------------------------------------------------------
    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_linear_ = msg.linear.x
        self.cmd_vel_angular_ = msg.angular.z
        self.cmd_vel_stamp_ = self.get_clock().now()

    def _update_gyro_bias(self, vyaw: float) -> None:
        """只在"机器人确实停着"时更新零偏估计。

        判定条件：已指令停止（或有 cmd_vel 但为 0）且 测到的 vyaw 足够小。
        慢速原地旋转时 vyaw 远大于 stationary_gyro_threshold，
        所以不会把真实转动当成零偏抹掉。
        """
        now = self.get_clock().now()
        dt = 0.0
        if self.last_stamp_ is not None:
            dt = (now - self.last_stamp_).nanoseconds * 1e-9
        self.last_stamp_ = now
        if dt <= 0.0:
            return

        if self.cmd_vel_stamp_ is None:
            # 完全没收到过 cmd_vel：不敢轻易判定"停着"，用更长时间窗
            cmd_stopped = abs(vyaw) < self.stationary_gyro_thr_
            need = self.stationary_delay_no_cmd_
        else:
            cmd_age = (now - self.cmd_vel_stamp_).nanoseconds * 1e-9
            if cmd_age < 1.0:
                cmd_stopped = (abs(self.cmd_vel_linear_) < self.stationary_vel_thr_
                               and abs(self.cmd_vel_angular_) < self.stationary_vel_thr_
                               and abs(vyaw) < self.stationary_gyro_thr_)
            else:
                # cmd_vel 停了（遥控松开后不再发布）
                cmd_stopped = abs(vyaw) < self.stationary_gyro_thr_
            need = self.stationary_delay_

        if cmd_stopped:
            self.stationary_time_ += dt
        else:
            self.stationary_time_ = 0.0
            return

        if self.stationary_time_ < need:
            return

        # 滑动平均逼近零偏
        alpha = min(1.0, dt / self.bias_tc_) if self.bias_tc_ > 0.0 else 1.0
        self.bias_ += alpha * (vyaw - self.bias_)
        if not self.bias_ready_:
            self.bias_ready_ = True
            self.get_logger().info(
                f'陀螺零偏估计就绪：{self.bias_:+.4f} rad/s '
                f'({self.bias_ * 57.2958:+.2f} deg/s)，之后发布时扣除')

    def _report_bias(self) -> None:
        if self.bias_ready_:
            self.get_logger().info(
                f'当前陀螺零偏扣除量：{self.bias_:+.4f} rad/s '
                f'({self.bias_ * 57.2958:+.2f} deg/s)')


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # rclpy 在 Ctrl+C 竞争时可能抛 RuntimeError
        node.get_logger().warn(f'imu_repub 退出: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
