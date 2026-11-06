/**
 * odom2tf —— 降级方案：把 /odom 的位姿直接转成 odom -> base_footprint 的 TF
 *             （use_ekf:=false 时使用，替代 EKF）
 *
 * 为什么要 restamp（2026-10-29 补）：
 *   下位机 /odom 用的是下位机自己的时钟，实测比上位机旧 0.4~0.7 s。
 *   而 map->odom（lidar_loc / slam_toolbox）和 /scan_ts（scan_repub）都按
 *   上位机时钟打戳。原实现直接照抄 msg->header，于是 odom->base_footprint
 *   永远比“现在”旧半秒，tf2 做精确时刻查询时会报：
 *       "Lookup would require extrapolation into the future"
 *   （Nav2 的代价地图/控制器正是按传感器时间戳做精确查询的）。
 *   这里增加 restamp 参数（默认 true），按本机接收时刻重新打戳，并把原始
 *   时间戳与接收时刻的差值打进日志，便于和 use_ekf:=true 对比。
 *   设 restamp:=false 可退回原来的“原样转发”，用于 A/B 排查。
 */
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomTopic2TF : public rclcpp::Node {
public:
  OdomTopic2TF(std::string name) : Node(name) {
    // 按本机接收时刻重打时间戳（原因见文件头注释）
    restamp_ = this->declare_parameter<bool>("restamp", true);

    // 订阅 odom 话题，使用传感器数据的 QoS
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&OdomTopic2TF::odom_callback_, this, std::placeholders::_1));
    // TF 广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(),
                "odom2tf: /odom -> TF odom->base_footprint, restamp = %s",
                restamp_ ? "true" : "false");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool restamp_{true};
  bool has_last_stamp_{false};
  bool logged_input_stamp_{false};
  // 三参数构造 (sec, nanosec, clock_type)，避免与 (nanosec, clock_type) 重载歧义
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

  // 回调：将里程计数据发布为 TF
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = msg->pose.pose.orientation;

    if (restamp_) {
      const rclcpp::Time now = this->now();

      // 第一次收到数据时报告下位机时间戳偏差，便于和 EKF 模式对比
      if (!logged_input_stamp_) {
        logged_input_stamp_ = true;
        const double age =
            (now - rclcpp::Time(msg->header.stamp, RCL_ROS_TIME)).seconds();
        RCLCPP_INFO(this->get_logger(),
                    "收到 /odom，原始时间戳比接收时刻旧 %.0f ms"
                    "（已按接收时刻重打戳）",
                    age * 1000.0);
      }

      rclcpp::Time stamp = now;
      // 保证严格单调递增，避免 tf2 报 data in the past/future 抖动
      if (has_last_stamp_ && stamp <= last_stamp_) {
        stamp = last_stamp_ + rclcpp::Duration(0, 1);
      }
      has_last_stamp_ = true;
      last_stamp_ = stamp;
      transform.header.stamp = stamp;
    }

    tf_broadcaster_->sendTransform(transform);
  };
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTopic2TF>("odom2tf");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
