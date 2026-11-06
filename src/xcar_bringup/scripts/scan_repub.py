#!/usr/bin/python3
"""
scan_repub.py —— 把 /scan 的时间戳对齐到上位机时钟，转发成 /scan_ts

为什么需要（2026-10-28 诊断结论）：
  下位机发布的 /scan 用的是**下位机自己的时钟**，实测比上位机时钟旧约 0.4~0.7 s。
  而上位机这边 odom->base_footprint（EKF）、map->odom（AMCL）都按上位机时钟打戳，
  于是 tf2 查询到处报：
      "Lookup would require extrapolation into the future"
      "Transform data too old when converting from odom to map"
  表现就是：
      * RViz 里点云/机器人/代价地图按过期位姿摆放（旋转时最明显，一片"糊"在墙上）
      * controller_server 偶尔拿不到 map 位姿，直接放弃该控制周期
        -> DWBLocalPlanner "No valid trajectories"
  注意：AMCL 发布 map->odom 的时间戳是 `scan_stamp + transform_tolerance`，
  所以 scan 的时间戳偏旧多少，map->odom 就跟着偏旧多少。
  把 transform_tolerance 调大只是"往后打戳"掩盖，代价是位姿滞后同样的时间。

做法：
  收到 /scan 后把 header.stamp 改成本机接收时刻（并保证严格单调递增），
  原样转发到 /scan_ts；Nav2 的 AMCL 与两个代价地图改订阅 /scan_ts。
  这样所有 TF 与传感器数据都在同一个时钟上。

  2026-10-30 修正（治本）：下位机是转镜式雷达，扫一圈 200ms（scan_time=0.2），
  固件把时间戳打在"扫描开始"时刻；我们若用"接收时刻"（≈扫描结束）重打戳，
  就等于把时间戳往后推了一整圈，而内容平均比它早半圈（100ms）。
  旋转时 角速度×100ms 的角度错位会把每个点画偏，整圈糊成一片。
  所以：重打戳时再减去 scan_time/2，把时间戳对齐到"扫描中点"。

参数：
  input_topic   原始激光话题（默认 scan）
  output_topic  重打戳后的话题（默认 scan_ts）
  restamp       false 则只转发、不改时间戳（用于 A/B 对比排查）
  compensate_scan_time   true 则把时间戳再对齐到扫描中点（减去 scan_time/2）
  report_period 日志里报告"原始时间戳与接收时刻差"的周期(秒)，0 = 不报
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class ScanRepub(Node):
    def __init__(self):
        super().__init__('scan_repub')

        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_ts')
        self.declare_parameter('restamp', True)
        self.declare_parameter('compensate_scan_time', True)
        self.declare_parameter('report_period', 5.0)
        # RViz / ros2 topic echo 默认用 Reliable 订阅，而 Best Effort 发布者与
        # Reliable 订阅者不兼容（表现为“话题选得到但一个点都没有”）。
        # 置 True 改用 Reliable 发布，两类订阅者都能收（7Hz 激光带宽很小）。
        self.declare_parameter('reliable', False)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.restamp_ = self.get_parameter('restamp').value
        self.compensate_scan_time_ = self.get_parameter('compensate_scan_time').value
        report_period = self.get_parameter('report_period').value
        reliable = self.get_parameter('reliable').value

        self.last_stamp_ = None
        self.offset_sum_ = 0.0
        self.offset_cnt_ = 0

        self.get_logger().info(
            f'scan_repub: {in_topic} -> {out_topic}, '
            f'restamp = {self.restamp_}, '
            f'compensate_scan_time = {self.compensate_scan_time_}, '
            f'qos = {"RELIABLE" if reliable else "BEST_EFFORT"}')

        if reliable:
            pub_qos = QoSProfile(
                depth=5,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE)
        else:
            pub_qos = qos_profile_sensor_data
        # 订阅端仍用 sensor_data QoS(BEST_EFFORT)，与下位机发布端一致
        self.pub_ = self.create_publisher(LaserScan, out_topic, pub_qos)
        self.sub_ = self.create_subscription(
            LaserScan, in_topic, self.callback, qos_profile_sensor_data)

        if report_period > 0.0:
            self.create_timer(report_period, self.report_offset)

    def callback(self, msg: LaserScan):
        now = self.get_clock().now()

        # 统计原始时间戳与本机接收时刻的差（下位机时钟偏差 + 传输延迟）
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            self.offset_sum_ += (now - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
            self.offset_cnt_ += 1

        if self.restamp_:
            stamp = now
            # 治本：把时间戳对齐到“扫描中点”而不是接收时刻（≈扫描结束）。
            # 转镜式雷达扫一圈要 scan_time 秒，内容平均比接收时刻早半圈；
            # 不减的话旋转时每个点都会被画偏 角速度×scan_time/2。
            if self.compensate_scan_time_ and msg.scan_time > 0.0:
                half_sweep = Duration(seconds=msg.scan_time / 2.0)
                stamp = stamp - half_sweep
            # 保证严格单调递增，避免 tf2 "data in the past/future" 抖动
            if self.last_stamp_ is not None and stamp <= self.last_stamp_:
                stamp = self.last_stamp_ + Duration(nanoseconds=1)
            self.last_stamp_ = stamp
            msg.header.stamp = stamp.to_msg()

        self.pub_.publish(msg)

    def report_offset(self):
        if self.offset_cnt_ == 0:
            return
        avg = self.offset_sum_ / self.offset_cnt_
        self.get_logger().info(
            f'原始 /scan 时间戳比接收时刻平均旧 {avg * 1000:.0f} ms'
            f'（最近 {self.offset_cnt_} 帧）；'
            f'{"已按接收时刻重打戳" if self.restamp_ else "未改戳"}')
        self.offset_sum_ = 0.0
        self.offset_cnt_ = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanRepub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # rclpy 在 Ctrl+C 竞争时可能抛 RuntimeError
        node.get_logger().warn(f'scan_repub 退出: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
