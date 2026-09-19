#!/usr/bin/python3
"""
imu_repub.py —— 把下位机的 /imu 转成 EKF 能正确使用的 /imu_rotated

注意：这里用 /usr/bin/python3 而不是 /usr/bin/env python3。
ROS2 Humble 的 rclpy 只对系统 Python 3.10 有效，如果环境里装了 conda / pyenv，
`env python3` 可能解析到别的版本（如 3.14），节点会直接 ModuleNotFoundError。

做两件事：
  1. 把 header.frame_id 改成 imu_link
     这样 robot_localization 会通过 TF 树上 imu_link 的旋转，
     自动补偿 MPU 的实际安装朝向（见 xcar_description/urdf/xcar.urdf）
  2. 协方差兜底
     协方差全 0 会让 EKF 无法正确加权（甚至导致融合失效），
     所以只在"全 0"时填一组默认值；如果下位机已经给了真实协方差，就原样保留。

参数：
  frame_id               目标 frame（默认 imu_link）
  fill_zero_covariance   是否在协方差全 0 时填默认值（默认 true）
  orientation_variance / angular_velocity_variance  默认协方差数值
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuRepub(Node):
    def __init__(self):
        super().__init__('imu_repub')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('fill_zero_covariance', True)
        self.declare_parameter('orientation_variance', 0.001)      # rad^2
        self.declare_parameter('angular_velocity_variance', 0.01)  # (rad/s)^2

        self.frame_id_ = self.get_parameter('frame_id').value
        self.fill_zero_ = self.get_parameter('fill_zero_covariance').value
        self.ori_var_ = self.get_parameter('orientation_variance').value
        self.ang_var_ = self.get_parameter('angular_velocity_variance').value

        self.get_logger().info(
            f'imu_repub: /imu -> /imu_rotated, frame_id = {self.frame_id_}, '
            f'fill_zero_covariance = {self.fill_zero_}')

        # 使用 sensor_data QoS (BEST_EFFORT)，与下位机发布 IMU 的 QoS 匹配
        self.pub = self.create_publisher(Imu, 'imu_rotated', qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Imu, 'imu', self.callback, qos_profile_sensor_data)

    @staticmethod
    def _is_all_zero(cov):
        return all(v == 0.0 for v in cov)

    def callback(self, msg: Imu):
        msg.header.frame_id = self.frame_id_

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
