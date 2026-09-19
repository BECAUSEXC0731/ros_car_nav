#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomTopic2TF : public rclcpp::Node {
public:
  OdomTopic2TF(std::string name) : Node(name) {
    // 订阅 odom 话题，使用传感器数据的 QoS
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&OdomTopic2TF::odom_callback_, this, std::placeholders::_1));
    // TF 广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 回调：将里程计数据发布为 TF
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = msg->pose.pose.orientation;
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
