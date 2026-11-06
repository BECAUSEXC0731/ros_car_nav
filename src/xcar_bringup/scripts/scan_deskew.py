#!/usr/bin/python3
"""scan_deskew.py —— 转镜雷达运动畸变补偿（去畸变）

背景（为什么需要这个节点）：
  转镜雷达 5Hz 扫一圈（scan_time≈0.2s，224 束，time_increment≈893us），
  每束点是在一圈内不同时刻采到的。车旋转/移动时，这 0.2s 里车自身的位姿一直在变，
  而 LaserScan 只有一个时间戳（scan_repub 已把它对齐到“扫描中点”t_mid）。
  Nav2 的 obstacle_layer 用这单一时间戳去变换全部 224 个点 → 一整圈被“糊”开：
    - 墙被拉成一整块脏数据
    - 墙上的点被投到原本没障碍的空地（表现为“空地上偶发障碍”）

  本节点按每束点的真实采样时刻 t_i = t_mid + (i - N/2)*time_increment，
  用 tf2 查 t_i 时刻 laser->odom 的位姿，把每束点纠正回“t_mid 时刻的激光坐标系”，
  输出一个去畸变后的 PointCloud2（frame 与输入 LaserScan 相同，stamp=t_mid）。
  代价地图障碍层改用这个点云后，旋转时不再糊成一片。

参数：
  input_topic    输入激光话题（默认 scan_ts，已重打戳）
  output_topic   输出去畸变点云话题（默认 scan_cloud）
  world_frame    机器人位姿所在的世界坐标系（默认 odom）
  robot_frame    机器人本体坐标系（默认 base_footprint，用于取静态 laser->base）
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def q_mul(a, b):
    """四元数乘法 a ⊗ b，元素顺序 (x, y, z, w)"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by + ay * bw + az * bx - ax * bz,
            aw * bz + az * bw + ax * by - ay * bx,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rot(q, v):
    """用四元数 q 旋转向量 v（v' = q ⊗ v ⊗ q⁻¹）"""
    vq = (v[0], v[1], v[2], 0.0)
    r = q_mul(q, q_mul(vq, q_conj(q)))
    return (r[0], r[1], r[2])


def q_nlerp(qa, qb, f):
    """四元数归一化线性插值（小角度足够精确）"""
    bx, by, bz, bw = qb
    dot = qa[0] * bx + qa[1] * by + qa[2] * bz + qa[3] * bw
    if dot < 0:
        bx, by, bz, bw = -bx, -by, -bz, -bw
        dot = -dot
    x = qa[0] + (bx - qa[0]) * f
    y = qa[1] + (by - qa[1]) * f
    z = qa[2] + (bz - qa[2]) * f
    w = qa[3] + (bw - qa[3]) * f
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return qa
    return (x / n, y / n, z / n, w / n)


class ScanDeskew(Node):
    def __init__(self):
        super().__init__('scan_deskew')
        self.declare_parameter('input_topic', 'scan_ts')
        self.declare_parameter('output_topic', 'scan_cloud')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.world_frame_ = self.get_parameter('world_frame').value
        self.robot_frame_ = self.get_parameter('robot_frame').value

        self.tf_buf_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buf_, self)
        self.pub_ = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.sub_ = self.create_subscription(
            LaserScan, in_topic, self.on_scan, qos_profile_sensor_data)
        self.drop_count_ = 0
        self.get_logger().info(
            f'scan_deskew: {in_topic} -> {out_topic} '
            f'(world={self.world_frame_}, robot={self.robot_frame_})')

    def _warn_drop(self, e):
        """TF 查询失败时丢弃本帧；每 25 帧打一条 WARN，避免刷屏又保持可见。"""
        self.drop_count_ += 1
        if self.drop_count_ % 25 == 1:
            self.get_logger().warn(
                f'TF 查询失败，已丢弃 {self.drop_count_} 帧，最近一次原因: {e}')

    @staticmethod
    def _tf_xyzq(tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def on_scan(self, scan):
        n = len(scan.ranges)
        if n == 0:
            return

        scan_time = scan.scan_time if scan.scan_time > 0.0 else 0.2
        half = Duration(seconds=scan_time / 2.0)
        # 扫描中点：直接用 scan_repub 打的时间戳（= 接收时刻 - 半圈），这是内容真实的中点。
        # 关键：不能锚到“最新 TF 的时间戳”——odom->base 的 TF 比真实时刻旧 14~123ms，
        # 锚过去会让点云整体带几十 ms 的时间戳偏移，旋转时按“角速度×偏移”整体转偏，
        # 停稳后旧标记被重新观测、反向转回来（正是“旋转拖影、停稳反向回正”现象）。
        t_mid = Time.from_msg(scan.header.stamp)

        try:
            # 静态 laser -> base（任何时刻都行，取最新）
            tf_bl = self.tf_buf_.lookup_transform(
                self.robot_frame_, scan.header.frame_id, Time())
            # 动态 odom -> base：
            #   起点 = t_mid - 半圈（一定在过去）
            #   终点 = 最新可用（≈ t_mid + 半圈；TF 滞后时直接查 t_mid+半圈会外推失败，故取最新）
            tf_ob_start = self.tf_buf_.lookup_transform(
                self.world_frame_, self.robot_frame_, t_mid - half)
            tf_ob_end = self.tf_buf_.lookup_transform(
                self.world_frame_, self.robot_frame_, Time())
        except Exception as e:  # noqa: BLE001 —— TF 还没就绪就丢这一帧
            self._warn_drop(e)
            return

        # 插值时间区间（终点用最新 TF，可能比 t_mid+半圈略早，最后几束钳到终点即可）
        t_start_s = tf_ob_start.header.stamp.sec + tf_ob_start.header.stamp.nanosec * 1e-9
        t_end_s = tf_ob_end.header.stamp.sec + tf_ob_end.header.stamp.nanosec * 1e-9
        t_mid_s = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        time_inc = scan.time_increment if scan.time_increment > 0.0 else scan_time / max(1, n)

        t_bl, q_bl = self._tf_xyzq(tf_bl)
        t_ob_s, q_ob_s = self._tf_xyzq(tf_ob_start)
        t_ob_e, q_ob_e = self._tf_xyzq(tf_ob_end)
        # 中点不单独查 TF（TF 滞后时 t_mid 可能落到“未来”），改用起终点按时间插值
        f_mid = (t_mid_s - t_start_s) / max(1e-9, t_end_s - t_start_s)
        f_mid = max(0.0, min(1.0, f_mid))
        t_ob_m = (t_ob_s[0] + (t_ob_e[0] - t_ob_s[0]) * f_mid,
                  t_ob_s[1] + (t_ob_e[1] - t_ob_s[1]) * f_mid,
                  t_ob_s[2] + (t_ob_e[2] - t_ob_s[2]) * f_mid)
        q_ob_m = q_nlerp(q_ob_s, q_ob_e, f_mid)

        # T_odom_laser(t_mid) = T_odom_base(t_mid) ∘ T_base_laser
        q_ol_m = q_mul(q_ob_m, q_bl)
        r_bl = q_rot(q_ob_m, t_bl)
        t_ol_m = (t_ob_m[0] + r_bl[0], t_ob_m[1] + r_bl[1], t_ob_m[2] + r_bl[2])
        # 其逆：laser(t_mid) <- odom
        q_ol_m_inv = q_conj(q_ol_m)

        pts = []
        for i in range(n):
            r = scan.ranges[i]
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            # 该束点真实采样时刻 t_i = t_mid + (i - (n-1)/2)*time_increment，按它在
            # [t_start, t_end] 上的位置插值（时间归一化，而非简单按索引）
            t_i = t_mid_s + (i - (n - 1) / 2.0) * time_inc
            f = (t_i - t_start_s) / max(1e-9, t_end_s - t_start_s)
            f = max(0.0, min(1.0, f))

            # 插值出 t_i 时刻 odom->base
            ti = (t_ob_s[0] + (t_ob_e[0] - t_ob_s[0]) * f,
                  t_ob_s[1] + (t_ob_e[1] - t_ob_s[1]) * f,
                  t_ob_s[2] + (t_ob_e[2] - t_ob_s[2]) * f)
            qi = q_nlerp(q_ob_s, q_ob_e, f)

            # 该束点在激光系里的坐标
            a = scan.angle_min + i * scan.angle_increment
            p_laser = (r * math.cos(a), r * math.sin(a), 0.0)

            # p_odom = T_odom_laser(t_i) · p_laser
            p_base = q_rot(q_bl, p_laser)
            p_base = (p_base[0] + t_bl[0], p_base[1] + t_bl[1], p_base[2] + t_bl[2])
            p_odom = q_rot(qi, p_base)
            p_odom = (p_odom[0] + ti[0], p_odom[1] + ti[1], p_odom[2] + ti[2])

            # 纠回 laser(t_mid)：p_laser_mid = T_odom_laser(t_mid)⁻¹ · p_odom
            d = (p_odom[0] - t_ol_m[0], p_odom[1] - t_ol_m[1], p_odom[2] - t_ol_m[2])
            p_mid = q_rot(q_ol_m_inv, d)
            pts.append([p_mid[0], p_mid[1], p_mid[2]])

        if not pts:
            return

        header = scan.header
        header.frame_id = scan.header.frame_id  # 保持与输入相同（激光系）
        header.stamp = t_mid.to_msg()           # 点云参考时刻 = 扫描真实中点（= scan_ts 的时间戳）
        cloud = point_cloud2.create_cloud_xyz32(header, pts)
        self.pub_.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ScanDeskew()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
