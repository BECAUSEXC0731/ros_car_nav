// 纯激光地图匹配定位节点（ROS2 Humble）—— 替代 Nav2 的 AMCL
//
// 算法来源：jie_ware/lidar_loc（ROS1 catkin，作者 Zhang Wanjie），移植到 ROS2 并修正：
//   1. ROS1 → ROS2：ros::NodeHandle → rclcpp、tf2 API、QoS、参数声明
//   2. 修正原实现的几何不一致：原代码把激光点 y 取反后，又用 `lidar_y - rotated_y`
//      做匹配，等价于把"旋转引起的位置耦合"镜像了一次。yaw=0 时正确，yaw 越大
//      匹配点越偏，旋转过程中会越跑越歪。这里统一用标准刚体变换：
//          x_map = cos(yaw) * x_base - sin(yaw) * y_base + X
//          y_map = sin(yaw) * x_base + cos(yaw) * y_base + Y
//   3. 梯度场由"逐障碍物贴 101x101 掩模取 max"（O(障碍数×101²)）改为
//      一次 distanceTransform（O(W·H)），结果等价、地图加载快一个量级
//   4. 增加工程健壮性：迭代上限、单帧最大修正量、匹配质量门槛、粗搜索全局重定位、
//      位姿话题输出、参数化
//
// 位姿约定：pose_* 表示 base_frame 在 map 坐标系下的位姿，与 AMCL 一致。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

// 一个候选位姿及其匹配得分
struct PoseCandidate
{
  double score;
  double x;
  double y;
  double yaw;
};
}  // namespace

class LidarLoc : public rclcpp::Node
{
public:
  LidarLoc()
  : rclcpp::Node("lidar_loc")
  {
    declareParameters();
    setupTf();
    setupCommunication();

    RCLCPP_INFO(
      get_logger(),
      "lidar_loc 启动：%s → %s（base=%s, odom=%s, 匹配话题=%s, TF=%s）",
      scan_topic_.c_str(), global_frame_.c_str(), base_frame_.c_str(),
      odom_frame_.c_str(), scan_topic_.c_str(), publish_tf_ ? "开启" : "关闭");
    if (!laser_frame_.empty()) {
      RCLCPP_INFO(get_logger(), "激光坐标系强制指定为 %s（忽略 scan.header.frame_id）",
        laser_frame_.c_str());
    }
  }

private:
  // ---------------------------------------------------------------- 参数
  void declareParameters()
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "scan_ts");
    // 去畸变点云（scan_deskew 输出，激光系）：旋转/移动时单时间戳的 LaserScan
    // 会被“糊”开，匹配跟着变噪。改用去畸变点云能显著压低移动时的位姿抖动。
    scan_cloud_topic_ = declare_parameter<std::string>("scan_cloud_topic", "scan_cloud");
    use_deskewed_scan_ = declare_parameter<bool>("use_deskewed_scan", true);
    map_topic_ = declare_parameter<std::string>("map_topic", "map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "amcl_pose");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    tf_rate_ = declare_parameter<double>("tf_rate", 30.0);

    // 初始位姿
    set_initial_pose_ = declare_parameter<bool>("set_initial_pose", true);
    init_x_ = declare_parameter<double>("initial_pose.x", 0.0);
    init_y_ = declare_parameter<double>("initial_pose.y", 0.0);
    init_yaw_ = declare_parameter<double>("initial_pose.yaw", 0.0);

    // 粗搜索全局重定位（没有初始位姿时用）
    global_relocalize_ = declare_parameter<bool>("global_relocalize", true);
    global_search_step_ = declare_parameter<double>("global_search_step", 0.30);
    global_yaw_step_ = declare_parameter<double>("global_yaw_step", 0.2618);  // 15°
    global_max_points_ = declare_parameter<int>("global_max_points", 120);
    global_candidates_ = declare_parameter<int>("global_candidates", 5);
    ambiguity_tolerance_ = declare_parameter<double>("ambiguity_tolerance", 0.01);

    // 匹配器
    // 评分场是“到最近障碍物距离”的高斯函数：field = 255*exp(-d²/(2σ²))，
    // 超过 gradient_radius 记 0。σ 越小峰越尖，匹配越精确、抗抖动越强；
    // 太大（如 2.5m 的线性渐变）会让得分面变平，静止时位姿在相邻像素/角度间来回跳。
    field_sigma_ = declare_parameter<double>("field_sigma", 0.30);
    gradient_radius_ = declare_parameter<double>("gradient_radius", 1.0);
    search_step_ratio_ = declare_parameter<double>("search_step_ratio", 1.0);
    yaw_step_deg_ = declare_parameter<double>("yaw_step_deg", 1.0);
    refine_ratio_ = declare_parameter<double>("refine_ratio", 0.2);
    refine_iterations_ = declare_parameter<int>("refine_iterations", 40);
    max_iterations_ = declare_parameter<int>("max_iterations", 400);
    min_quality_ = declare_parameter<double>("min_quality", 0.15);
    min_points_ = declare_parameter<int>("min_points", 40);
    max_correction_trans_ = declare_parameter<double>("max_correction_trans", 0.5);
    max_correction_rot_ = declare_parameter<double>("max_correction_rot", 0.4);
    smoothing_alpha_ = declare_parameter<double>("smoothing_alpha", 0.5);
    deadband_trans_ = declare_parameter<double>("deadband_trans", 0.02);
    deadband_yaw_ = declare_parameter<double>("deadband_yaw_deg", 0.5) * kDeg2Rad;
    max_scan_points_ = declare_parameter<int>("max_scan_points", 360);
    log_period_ = declare_parameter<double>("log_period", 5.0);

    // 重定位（点选初值 / 粗搜索）后主动清空两个代价地图：
    // 旧位姿下标记的障碍会被留在滚动窗口里，而被墙挡住的区域永远没有射线去清除，
    // 表现为局部代价地图里“擦不掉的一大块”。
    clear_costmaps_ = declare_parameter<bool>("clear_costmaps_on_relocalize", true);
    global_clear_service_ = declare_parameter<std::string>(
      "global_clear_service", "/global_costmap/clear_entirely_global_costmap");
    local_clear_service_ = declare_parameter<std::string>(
      "local_clear_service", "/local_costmap/clear_entirely_local_costmap");
    // 静止（位姿稳定）超过这么多秒就自动清空一次代价地图。0 = 关闭。
    // 为什么需要：局部代价地图是滚动窗口，会把“走过的地方看到过的障碍”留着，
    // 被墙挡住的区域没有射线去清除，于是屏幕上一直留着一大块。停车后自动清理
    // 可以当作“重新看一遍”——清掉后下一帧激光会把真正看得见的障碍重新标回来。
    clear_when_still_sec_ = declare_parameter<double>("clear_costmaps_when_still_sec", 0.0);
  }

  // ---------------------------------------------------------------- 通信
  void setupTf()
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void setupCommunication()
  {
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);

    // map_server 用 transient_local 发布地图，这里必须匹配，否则晚启动拿不到
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&LidarLoc::mapCallback, this, std::placeholders::_1));

    // 激光：默认用去畸变点云（scan_deskew 输出），可切回 LaserScan（use_deskewed_scan=false）
    // 两者都用 sensor_data QoS(BEST_EFFORT)，与发布端一致
    if (use_deskewed_scan_) {
      scan_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LidarLoc::scanCloudCallback, this, std::placeholders::_1));
    } else {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LidarLoc::scanCallback, this, std::placeholders::_1));
    }

    // RViz 的 "2D Pose Estimate" 按钮
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&LidarLoc::initialPoseCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tf_rate_));
    tf_timer_ = create_wall_timer(period, std::bind(&LidarLoc::publishTf, this));

    if (clear_costmaps_) {
      global_clear_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(global_clear_service_);
      local_clear_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(local_clear_service_);
    }
  }

  // 重定位后清空代价地图。Nav2 的两个 costmap 都提供 clear_entirely_* 服务，
  // 没起来（服务不可用）时静默跳过。
  void clearCostmaps(const char * reason)
  {
    if (!clear_costmaps_) {
      return;
    }
    auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    for (auto * client : {global_clear_client_.get(), local_clear_client_.get()}) {
      if (client && client->service_is_ready()) {
        client->async_send_request(req);
      }
    }
    RCLCPP_INFO(get_logger(), "已请求清空代价地图（%s）", reason);
  }

  // ---------------------------------------------------------------- 地图
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = *msg;
    if (map_msg_.info.resolution <= 0.0f || map_msg_.info.width == 0 || map_msg_.info.height == 0) {
      RCLCPP_ERROR(get_logger(), "地图无效（resolution=%f, %ux%u），忽略",
        map_msg_.info.resolution, map_msg_.info.width, map_msg_.info.height);
      return;
    }

    resolution_ = map_msg_.info.resolution;
    origin_x_ = map_msg_.info.origin.position.x;
    origin_y_ = map_msg_.info.origin.position.y;

    if (!buildSearchField()) {
      return;
    }
    map_ready_ = true;
    RCLCPP_INFO(
      get_logger(),
      "地图就绪：%ux%u @ %.3f m/格，origin=(%.2f, %.2f)；匹配场 %dx%d，"
      "梯度半径 %.2f m，场均值 %.1f/255",
      map_msg_.info.width, map_msg_.info.height, resolution_, origin_x_, origin_y_,
      map_field_.cols, map_field_.rows, gradient_radius_, cv::mean(map_field_)[0]);

    if (set_initial_pose_ && !pose_initialized_) {
      setPose(init_x_, init_y_, init_yaw_, "参数 initial_pose");
    }
  }

  // 把整张地图裁到障碍物包围盒（外扩梯度半径），并生成"到最近障碍物距离"的渐变场
  bool buildSearchField()
  {
    const int w = static_cast<int>(map_msg_.info.width);
    const int h = static_cast<int>(map_msg_.info.height);
    if (map_msg_.data.size() != static_cast<size_t>(w) * static_cast<size_t>(h)) {
      RCLCPP_ERROR(get_logger(), "地图数据长度 %zu 与 %dx%d 不符", map_msg_.data.size(), w, h);
      return false;
    }

    // 1) 占据格子二值图（255 = 障碍）；同时求包围盒
    cv::Mat occupied = cv::Mat::zeros(h, w, CV_8UC1);
    int xmin = w, xmax = -1, ymin = h, ymax = -1;
    for (int row = 0; row < h; ++row) {
      const int8_t * src = &map_msg_.data[static_cast<size_t>(row) * static_cast<size_t>(w)];
      uint8_t * dst = occupied.ptr<uint8_t>(row);
      for (int col = 0; col < w; ++col) {
        if (src[col] >= 100) {
          dst[col] = 255;
          xmin = std::min(xmin, col);
          xmax = std::max(xmax, col);
          ymin = std::min(ymin, row);
          ymax = std::max(ymax, row);
        }
      }
    }
    if (xmax < 0) {
      RCLCPP_ERROR(get_logger(), "地图里没有任何占据格子，无法定位");
      return false;
    }

    // 2) 裁剪（外扩梯度半径，保证边界处的点也能被评分）
    const int margin = std::max(1, static_cast<int>(std::lround(gradient_radius_ / resolution_)));
    const int x0 = std::max(0, xmin - margin);
    const int y0 = std::max(0, ymin - margin);
    const int x1 = std::min(w - 1, xmax + margin);
    const int y1 = std::min(h - 1, ymax + margin);
    const int cw = x1 - x0 + 1;
    const int ch = y1 - y0 + 1;

    roi_x_ = x0;  // 裁剪窗口在整张地图里的像素偏移
    roi_y_ = y0;

    cv::Mat occ_crop = occupied(cv::Rect(x0, y0, cw, ch)).clone();

    // 3) 距离变换：障碍=0，其它=255 → 每个像素到最近障碍的像素距离
    cv::Mat free_space;
    cv::bitwise_not(occ_crop, free_space);
    cv::Mat dist;
    cv::distanceTransform(free_space, dist, cv::DIST_L2, 3);

    // 4) 渐变场：高斯似然场 field = 255*exp(-d²/(2σ²))，超过半径记 0。
    //    要点：必须是“离墙越远迅速降为 0”的尖峰。原来那种 2.5m 线性渐变在
    //    小房间里整个得分面几乎是平的（场均值 200/255），静止时匹配结果会在
    //    相邻像素/角度间来回跳，RViz 里就是高频抖动、代价地图上是一片噪点。
    const double sigma_px = std::max(1e-3, field_sigma_ / resolution_);
    const double radius_px = std::max(sigma_px, gradient_radius_ / resolution_);
    cv::Mat field(ch, cw, CV_8UC1);
    field.setTo(0);
    for (int row = 0; row < ch; ++row) {
      const float * d = dist.ptr<float>(row);
      uint8_t * f = field.ptr<uint8_t>(row);
      for (int col = 0; col < cw; ++col) {
        const double dp = static_cast<double>(d[col]);
        if (dp > radius_px) {
          continue;
        }
        const double g = std::exp(-0.5 * dp * dp / (sigma_px * sigma_px));
        f[col] = static_cast<uint8_t>(cv::saturate_cast<uchar>(255.0 * g));
      }
    }

    map_field_ = field;
    return true;
  }

  // ---------------------------------------------------------------- 位姿设置
  void setPose(double x, double y, double yaw, const char * reason)
  {
    pose_x_ = x;
    pose_y_ = y;
    pose_yaw_ = normalizeAngle(yaw);
    pose_initialized_ = true;
    matched_scans_ = 0;
    // 位姿被外部改写（初值 / 2D Pose Estimate / 全局重定位）后，里程计参考要作废，
    // 否则下一次会拿"旧位姿 + 这段里程计"去做预测
    have_odom_ref_ = false;
    RCLCPP_INFO(
      get_logger(), "初始化位姿（%s）：x=%.3f y=%.3f yaw=%.1f°",
      reason, pose_x_, pose_y_, pose_yaw_ / kDeg2Rad);
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!map_ready_) {
      RCLCPP_WARN(get_logger(), "还没收到地图，忽略 initialpose");
      return;
    }
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;
    setPose(
      msg->pose.pose.position.x, msg->pose.pose.position.y, yaw,
      "RViz 2D Pose Estimate / initialpose");
    clearCostmaps("手工重定位");
  }

  // ---------------------------------------------------------------- 激光
  // 查 laser -> base 的静态变换，并判断雷达是否倒装（roll≈180°）
  bool lookupLaserToBase(const std::string & frame, tf2::Transform & laser_to_base, bool & inverted)
  {
    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "激光消息没有 frame_id，且未设置 laser_frame 参数");
      return false;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      // 用最新可用变换（激光是 z=0 的 2D 扫描，静态 TF 即可）
      tf_msg = tf_buffer_->lookupTransform(
        base_frame_, frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "查不到 %s -> %s：%s",
        frame.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    // 倒装雷达（roll≈180°）：点的 x/y 取反，与原实现一致
    tf2::Quaternion q_lidar;
    tf2::fromMsg(tf_msg.transform.rotation, q_lidar);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);
    (void)yaw;
    const double tol = 0.1;
    inverted =
      std::abs(std::abs(roll) - M_PI) < tol && !(std::abs(std::abs(pitch) - M_PI) < tol);

    tf2::fromMsg(tf_msg.transform, laser_to_base);
    return true;
  }

  // 把一帧激光从激光坐标系变到 base 坐标系（单位：米）
  bool scanToBase(
    const sensor_msgs::msg::LaserScan::SharedPtr & msg,
    std::vector<cv::Point2f> & pts)
  {
    const std::string frame = laser_frame_.empty() ? msg->header.frame_id : laser_frame_;
    tf2::Transform laser_to_base;
    bool inverted = false;
    if (!lookupLaserToBase(frame, laser_to_base, inverted)) {
      return false;
    }

    const size_t n = msg->ranges.size();
    pts.clear();
    pts.reserve(n);
    double angle = msg->angle_min;
    for (size_t i = 0; i < n; ++i, angle += msg->angle_increment) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
        continue;
      }
      const tf2::Vector3 p_laser(r * std::cos(angle), r * std::sin(angle), 0.0);
      const tf2::Vector3 p_base = laser_to_base * p_laser;
      double px = p_base.x();
      double py = p_base.y();
      if (inverted) {
        px = -px;
        py = -py;
      }
      pts.emplace_back(static_cast<float>(px), static_cast<float>(py));
    }
    return !pts.empty();
  }

  // 把去畸变点云（激光系，scan_deskew 输出）变到 base 坐标系。点云里的点已经是
  // “扫描中点时刻”的激光坐标，这里只做 laser->base 的静态变换，不再从 ranges 反算。
  bool cloudToBase(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    std::vector<cv::Point2f> & pts)
  {
    const std::string frame = laser_frame_.empty() ? msg->header.frame_id : laser_frame_;
    tf2::Transform laser_to_base;
    bool inverted = false;
    if (!lookupLaserToBase(frame, laser_to_base, inverted)) {
      return false;
    }

    pts.clear();
    pts.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      const float lx = *iter_x;
      const float ly = *iter_y;
      if (!std::isfinite(lx) || !std::isfinite(ly)) {
        continue;
      }
      const tf2::Vector3 p_laser(lx, ly, 0.0);
      const tf2::Vector3 p_base = laser_to_base * p_laser;
      double px = p_base.x();
      double py = p_base.y();
      if (inverted) {
        px = -px;
        py = -py;
      }
      pts.emplace_back(static_cast<float>(px), static_cast<float>(py));
    }
    return !pts.empty();
  }

  // 位姿 (x, y, yaw) 下，所有激光点在似然场里的取值之和（越大越贴合地图）。
  // 用双线性插值而不是“取最近像素”：最近像素采样会把目标函数变成 5cm 一级的台阶，
  // 细步长搜不到梯度，静止时位姿只能在相邻格之间随机走（实测 yaw 会持续游走）。
  double scorePose(const std::vector<cv::Point2f> & pts, double x, double y, double yaw) const
  {
    if (map_field_.empty() || pts.empty()) {
      return 0.0;
    }
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double inv_res = 1.0 / resolution_;
    const int cols = map_field_.cols;
    const int rows = map_field_.rows;

    double sum = 0.0;
    for (const auto & p : pts) {
      // base → map（标准刚体变换）
      const double xm = c * static_cast<double>(p.x) - s * static_cast<double>(p.y) + x;
      const double ym = s * static_cast<double>(p.x) + c * static_cast<double>(p.y) + y;
      const double fc = (xm - origin_x_) * inv_res - static_cast<double>(roi_x_);
      const double fr = (ym - origin_y_) * inv_res - static_cast<double>(roi_y_);
      if (fc < 0.0 || fr < 0.0 || fc >= cols - 1 || fr >= rows - 1) {
        continue;
      }
      const int c0 = static_cast<int>(fc);
      const int r0 = static_cast<int>(fr);
      const double tc = fc - static_cast<double>(c0);
      const double tr = fr - static_cast<double>(r0);
      const double v00 = map_field_.at<uint8_t>(r0, c0);
      const double v01 = map_field_.at<uint8_t>(r0, c0 + 1);
      const double v10 = map_field_.at<uint8_t>(r0 + 1, c0);
      const double v11 = map_field_.at<uint8_t>(r0 + 1, c0 + 1);
      sum += (1.0 - tc) * (1.0 - tr) * v00 + tc * (1.0 - tr) * v01 +
        (1.0 - tc) * tr * v10 + tc * tr * v11;
    }
    return sum;
  }

  // 爬山式搜索：每次在 5 个平移候选 × 3 个朝向候选里挑最优，直到不再改善
  double hillClimb(
    const std::vector<cv::Point2f> & pts, double & x, double & y, double & yaw,
    double step, double yaw_step, int max_iters) const
  {
    const std::vector<std::pair<double, double>> offsets = {
      {0.0, 0.0}, {step, 0.0}, {-step, 0.0}, {0.0, step}, {0.0, -step}};

    double best = scorePose(pts, x, y, yaw);
    for (int iter = 0; iter < max_iters; ++iter) {
      double best_cand = best;
      double best_dx = 0.0;
      double best_dy = 0.0;
      double best_dyaw = 0.0;
      for (const auto & off : offsets) {
        for (int j = 0; j < 3; ++j) {
          const double dyaw = (j == 0) ? 0.0 : ((j == 1) ? yaw_step : -yaw_step);
          const double s = scorePose(pts, x + off.first, y + off.second, yaw + dyaw);
          if (s > best_cand) {
            best_cand = s;
            best_dx = off.first;
            best_dy = off.second;
            best_dyaw = dyaw;
          }
        }
      }
      // 15 个候选都不比当前好（含原地不动这个候选）→ 收敛
      if (best_dx == 0.0 && best_dy == 0.0 && best_dyaw == 0.0) {
        break;
      }
      x += best_dx;
      y += best_dy;
      yaw += best_dyaw;
      best = best_cand;
    }
    return best;
  }

  // 两段式爬山：先用参数步长（默认 5cm / 1°）大范围收敛，
  // 再用 refine_ratio 倍的细步长（默认 1cm / 0.2°）精修。
  // 精修是为了消除“整像素/整度”的量化台阶——否则静止时匹配结果会在相邻格/相邻角度之间
  // 来回跳（实测表现为 RViz 里机器人高频抖动、yaw 在 170.1°/171.1° 之间刷）。
  void matchScan(
    const std::vector<cv::Point2f> & pts,
    double & x, double & y, double & yaw, double & quality) const
  {
    const double coarse_step = std::max(1e-3, search_step_ratio_ * resolution_);
    const double coarse_yaw = std::max(1e-4, yaw_step_deg_ * kDeg2Rad);
    hillClimb(pts, x, y, yaw, coarse_step, coarse_yaw, max_iterations_);

    const double ratio = std::clamp(refine_ratio_, 0.01, 1.0);
    hillClimb(
      pts, x, y, yaw, std::max(1e-4, ratio * coarse_step),
      std::max(1e-5, ratio * coarse_yaw), std::max(1, refine_iterations_));

    const double best = scorePose(pts, x, y, yaw);
    quality = pts.empty() ? 0.0 : best / (255.0 * static_cast<double>(pts.size()));
  }

  // 粗搜索全局重定位：在整个地图上按 global_search_step × global_yaw_step 枚举，
  // 再对空间上互不相同的 Top-K 候选逐个做完整爬山复核。
  //
  // 为什么不只取粗搜索的第一名：本算法的得分是"激光点到最近障碍的距离"，
  // 在对称/近似对称的房间里，180° 镜像位姿的粗得分经常只差零点几 %，
  // 一次 0.3m/15° 的粗搜索挑错就会整体反了。多复核几个候选能显著降低这种错误。
  bool globalRelocalize(const std::vector<cv::Point2f> & pts)
  {
    if (map_field_.empty() || pts.empty()) {
      return false;
    }
    const std::vector<cv::Point2f> sub = subsample(pts, global_max_points_);
    if (sub.empty()) {
      return false;
    }

    const int step_px = std::max(1, static_cast<int>(std::lround(global_search_step_ / resolution_)));
    const double yaw_step = std::max(1e-3, global_yaw_step_);
    const int yaw_bins = std::max(1, static_cast<int>(std::lround(2.0 * M_PI / yaw_step)));

    std::vector<PoseCandidate> candidates;
    for (int row = 0; row < map_field_.rows; row += step_px) {
      for (int col = 0; col < map_field_.cols; col += step_px) {
        const double wx = origin_x_ + (static_cast<double>(col + roi_x_) + 0.5) * resolution_;
        const double wy = origin_y_ + (static_cast<double>(row + roi_y_) + 0.5) * resolution_;
        for (int k = 0; k < yaw_bins; ++k) {
          const double yaw = -M_PI + static_cast<double>(k) * yaw_step;
          const double s = scorePose(sub, wx, wy, yaw);
          if (s > 0.0) {
            candidates.push_back(PoseCandidate{s, wx, wy, yaw});
          }
        }
      }
    }
    if (candidates.empty()) {
      return false;
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const PoseCandidate & a, const PoseCandidate & b) {return a.score > b.score;});

    // 空间上互不相同（>0.5m 或 >30°）的前 K 个候选
    std::vector<PoseCandidate> picked;
    for (const auto & c : candidates) {
      bool distinct = true;
      for (const auto & p : picked) {
        if (std::hypot(c.x - p.x, c.y - p.y) < 0.5 &&
          std::abs(normalizeAngle(c.yaw - p.yaw)) < 30.0 * kDeg2Rad)
        {
          distinct = false;
          break;
        }
      }
      if (distinct) {
        picked.push_back(c);
      }
      if (static_cast<int>(picked.size()) >= std::max(1, global_candidates_)) {
        break;
      }
    }

    std::vector<PoseCandidate> refined;
    refined.reserve(picked.size());
    for (const auto & c : picked) {
      double x = c.x;
      double y = c.y;
      double yaw = c.yaw;
      double q = 0.0;
      matchScan(pts, x, y, yaw, q);
      refined.push_back(PoseCandidate{q, x, y, yaw});
    }
    std::sort(
      refined.begin(), refined.end(),
      [](const PoseCandidate & a, const PoseCandidate & b) {return a.score > b.score;});

    const PoseCandidate & best = refined.front();
    if (best.score <= 0.0) {
      return false;
    }

    // 歧义检测：两个相差很远的候选得分几乎一样 → 房间很可能对称，需要人工给初值
    for (size_t i = 1; i < refined.size(); ++i) {
      const PoseCandidate & other = refined[i];
      const double dist = std::hypot(best.x - other.x, best.y - other.y);
      const double dth = std::abs(normalizeAngle(best.yaw - other.yaw)) / kDeg2Rad;
      if (best.score - other.score < ambiguity_tolerance_ && (dist > 0.5 || dth > 30.0)) {
        RCLCPP_WARN(
          get_logger(),
          "全局重定位存在歧义：最优 (%.2f, %.2f, %.0f°) 与次优 (%.2f, %.2f, %.0f°) "
          "得分几乎相同（%.3f / %.3f）。房间可能近似对称，请在 RViz 里点 "
          "“2D Pose Estimate”确认，或把 set_initial_pose 打开改用手工初值。",
          best.x, best.y, best.yaw / kDeg2Rad, other.x, other.y, other.yaw / kDeg2Rad,
          best.score, other.score);
        break;
      }
    }

    setPose(best.x, best.y, best.yaw, "粗搜索全局重定位");
    clearCostmaps("粗搜索重定位");
    RCLCPP_INFO(
      get_logger(), "粗搜索复核了 %zu 个候选，最优质量 %.3f（次优 %.3f）",
      refined.size(), best.score, refined.size() > 1 ? refined[1].score : 0.0);
    return true;
  }

  static std::vector<cv::Point2f> subsample(
    const std::vector<cv::Point2f> & pts, int max_points)
  {
    if (max_points <= 0 || pts.size() <= static_cast<size_t>(max_points)) {
      return pts;
    }
    const size_t stride =
      (pts.size() + static_cast<size_t>(max_points) - 1) / static_cast<size_t>(max_points);
    std::vector<cv::Point2f> out;
    out.reserve(static_cast<size_t>(max_points) + 1);
    for (size_t i = 0; i < pts.size(); i += stride) {
      out.push_back(pts[i]);
    }
    return out;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<cv::Point2f> pts;
    if (!scanToBase(msg, pts)) {
      return;
    }
    processPoints(pts, msg->header.stamp);
  }

  void scanCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<cv::Point2f> pts;
    if (!cloudToBase(msg, pts)) {
      return;
    }
    processPoints(pts, msg->header.stamp);
  }

  // 对一帧已经变换到 base 坐标系的激光点做匹配 + 平滑 + 发布（LaserScan / PointCloud2 共用）
  void processPoints(std::vector<cv::Point2f> pts, const rclcpp::Time & stamp)
  {
    if (!map_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "还没收到地图（%s），先不做定位", map_topic_.c_str());
      return;
    }
    if (max_scan_points_ > 0 && pts.size() > static_cast<size_t>(max_scan_points_)) {
      pts = subsample(pts, max_scan_points_);
    }
    // 点数太少时得分没有意义（实测出现过只有 2 个点却算成“质量 1.000”的情况）
    if (pts.size() < static_cast<size_t>(std::max(1, min_points_))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "本帧只有 %zu 个有效激光点（< %d），忽略", pts.size(), min_points_);
      return;
    }

    // 没有任何先验位姿时的兜底：粗搜索全局重定位
    if (!pose_initialized_) {
      if (!global_relocalize_ || global_relocalize_tried_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "等待初始位姿：请在 RViz 里点 “2D Pose Estimate”（或打开 global_relocalize）");
        return;
      }
      global_relocalize_tried_ = true;
      if (!globalRelocalize(pts)) {
        // 等下一帧再试
        global_relocalize_tried_ = false;
        return;
      }
    }

    // ---- 用里程计把上一帧位姿推到当前时刻，作为搜索起点 ----
    // T_map_base(new) = T_map_base(old) * (T_odom_base(old))⁻¹ * T_odom_base(new)
    tf2::Transform odom_base;
    const bool have_odom = lookupOdomBase(odom_base);
    double pred_x = pose_x_;
    double pred_y = pose_y_;
    double pred_yaw = pose_yaw_;
    if (have_odom && have_odom_ref_) {
      const tf2::Transform delta = odom_base_ref_.inverse() * odom_base;
      const tf2::Transform pred = mapBaseTransform(pose_x_, pose_y_, pose_yaw_) * delta;
      pred_x = pred.getOrigin().x();
      pred_y = pred.getOrigin().y();
      pred_yaw = yawOf(pred.getRotation());
    }

    double x = pred_x;
    double y = pred_y;
    double yaw = pred_yaw;
    double quality = 0.0;
    matchScan(pts, x, y, yaw, quality);

    // 匹配结果相对“预测位姿”的修正量；过大说明爬到了另一个局部最优
    const double corr = std::hypot(x - pred_x, y - pred_y);
    const double corr_yaw = std::abs(normalizeAngle(yaw - pred_yaw));
    if (corr > max_correction_trans_ || corr_yaw > max_correction_rot_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "匹配修正量过大（%.2f m / %.1f°），已丢弃本帧", corr, corr_yaw / kDeg2Rad);
      return;
    }

    // 匹配质量门槛：首帧（初始化那一帧）不做限制，否则永远起不来
    if (matched_scans_ > 0 && quality < min_quality_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "匹配质量过低（%.3f < %.3f），本帧不更新位姿", quality, min_quality_);
      return;
    }

    // ---- α 平滑 + 死区 ----
    // 预测里已经包含里程计运动，所以平滑不会把真实运动磨掉，只压掉逐帧匹配噪声。
    // 修正量小于死区时就认为“里程计足够准”，直接沿用预测位姿：静止时实现零抖动。
    // smoothing_alpha = 1.0 表示完全不平滑。
    if (corr < deadband_trans_ && corr_yaw < deadband_yaw_) {
      pose_x_ = pred_x;
      pose_y_ = pred_y;
      pose_yaw_ = pred_yaw;
    } else {
      const double alpha = std::clamp(smoothing_alpha_, 0.05, 1.0);
      pose_x_ = pred_x + alpha * (x - pred_x);
      pose_y_ = pred_y + alpha * (y - pred_y);
      pose_yaw_ = normalizeAngle(pred_yaw + alpha * normalizeAngle(yaw - pred_yaw));
    }

    if (have_odom) {
      odom_base_ref_ = odom_base;
      have_odom_ref_ = true;
    }
    ++matched_scans_;
    publishPose(stamp);
    publishTf();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), static_cast<int64_t>(std::max(0.1, log_period_) * 1000),
      "位姿 x=%.3f y=%.3f yaw=%.1f° | 质量 %.3f | 本帧修正 %.1f cm / %.2f° | %zu 点",
      pose_x_, pose_y_, pose_yaw_ / kDeg2Rad, quality, corr * 100.0, corr_yaw / kDeg2Rad,
      pts.size());

    // 静止自动清空（可选，默认关）：位姿稳定超过 clear_when_still_sec_ 秒就清一次，
    // 把“行驶历史”留下的残留块清掉；清掉后下一帧激光会把当前看得见的障碍标回来。
    if (clear_when_still_sec_ > 0.0) {
      const rclcpp::Time now_t = now();
      const bool moved =
        std::hypot(pose_x_ - still_x_, pose_y_ - still_y_) > 0.05 ||
        std::abs(normalizeAngle(pose_yaw_ - still_yaw_)) > 5.0 * kDeg2Rad;
      if (moved) {
        still_x_ = pose_x_;
        still_y_ = pose_y_;
        still_yaw_ = pose_yaw_;
        still_since_ = now_t;
        cleared_while_still_ = false;
      } else if (!cleared_while_still_ &&
        (now_t - still_since_).seconds() > clear_when_still_sec_)
      {
        clearCostmaps("静止自动清理");
        cleared_while_still_ = true;
      }
    }
  }

  // 取最新的 odom -> base 变换
  bool lookupOdomBase(tf2::Transform & out, double timeout_sec = 0.05)
  {
    try {
      const auto ts = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(timeout_sec));
      tf2::fromMsg(ts.transform, out);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  static tf2::Transform mapBaseTransform(double x, double y, double yaw)
  {
    tf2::Transform t;
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    t.setRotation(q);
    return t;
  }

  // 取平面 yaw（只关心绕 z 的旋转）
  static double yawOf(const tf2::Quaternion & q)
  {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  // ---------------------------------------------------------------- 输出
  void publishPose(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = global_frame_;
    msg.pose.pose.position.x = pose_x_;
    msg.pose.pose.position.y = pose_y_;
    msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose_yaw_);
    msg.pose.pose.orientation = tf2::toMsg(q);

    // 与 AMCL 风格一致的协方差占位（Nav2 目前不消费它，RViz 用来画椭圆）
    auto & cov = msg.pose.covariance;
    cov.fill(0.0);
    cov[0] = 0.02;
    cov[7] = 0.02;
    cov[35] = 0.1;
    pose_pub_->publish(msg);
  }

  void publishTf()
  {
    if (!publish_tf_ || !pose_initialized_ || !map_ready_) {
      return;
    }

    // 查询最新的 odom->base；查不到时用上一次的缓存继续发。
    // 绝不能因为一次查询失败就整周期不发布 map->odom：
    // 单线程节点在跑匹配时会让 TF 监听器积压，查询偶尔超时，
    // 一跳过就会让 RViz / 代价地图报
    // “Lookup would require extrapolation into the future”。
    tf2::Transform odom_to_base_tf;
    if (lookupOdomBase(odom_to_base_tf, 0.1)) {
      last_odom_base_ = odom_to_base_tf;
      have_last_odom_base_ = true;
    } else if (have_last_odom_base_) {
      odom_to_base_tf = last_odom_base_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "查不到 %s -> %s，暂用上一次的变换继续发布 map->odom",
        odom_frame_.c_str(), base_frame_.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "查不到 %s -> %s，且无缓存，本周期不发布 map->odom",
        odom_frame_.c_str(), base_frame_.c_str());
      return;
    }

    // T_map_odom = T_map_base * (T_odom_base)^-1
    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(pose_x_, pose_y_, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose_yaw_);
    map_to_base.setRotation(q);

    const tf2::Transform map_to_odom = map_to_base * odom_to_base_tf.inverse();

    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = now();
    out.header.frame_id = global_frame_;
    out.child_frame_id = odom_frame_;
    tf2::toMsg(map_to_odom, out.transform);
    tf_broadcaster_->sendTransform(out);
  }

  // ---------------------------------------------------------------- 成员
  // 参数
  std::string base_frame_;
  std::string odom_frame_;
  std::string global_frame_;
  std::string laser_frame_;
  std::string scan_topic_;
  std::string scan_cloud_topic_;
  bool use_deskewed_scan_{true};
  std::string map_topic_;
  std::string pose_topic_;
  bool publish_tf_{true};
  double tf_rate_{30.0};

  bool set_initial_pose_{true};
  double init_x_{0.0};
  double init_y_{0.0};
  double init_yaw_{0.0};

  bool global_relocalize_{true};
  double global_search_step_{0.30};
  double global_yaw_step_{0.2618};
  int global_max_points_{120};
  int global_candidates_{5};
  double ambiguity_tolerance_{0.01};

  double field_sigma_{0.30};
  double gradient_radius_{1.0};
  double search_step_ratio_{1.0};
  double yaw_step_deg_{1.0};
  double refine_ratio_{0.2};
  int refine_iterations_{40};
  int max_iterations_{400};
  double min_quality_{0.15};
  int min_points_{40};
  double max_correction_trans_{0.5};
  double max_correction_rot_{0.4};
  double smoothing_alpha_{0.5};
  double deadband_trans_{0.02};
  double deadband_yaw_{0.5 * M_PI / 180.0};
  int max_scan_points_{360};
  double log_period_{5.0};

  bool clear_costmaps_{true};
  std::string global_clear_service_;
  std::string local_clear_service_;
  double clear_when_still_sec_{0.0};
  double still_x_{0.0};
  double still_y_{0.0};
  double still_yaw_{0.0};
  rclcpp::Time still_since_{0, 0, RCL_ROS_TIME};
  bool cleared_while_still_{false};

  // 地图与匹配场
  nav_msgs::msg::OccupancyGrid map_msg_;
  cv::Mat map_field_;
  int roi_x_{0};
  int roi_y_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  bool map_ready_{false};

  // 位姿
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  bool pose_initialized_{false};
  bool global_relocalize_tried_{false};
  int matched_scans_{0};

  // 上一次匹配时的 odom->base（用来把位姿预测到当前时刻）
  tf2::Transform odom_base_ref_;
  bool have_odom_ref_{false};

  // 最近一次成功查询到的 odom->base（仅用于 TF 发布时兑底，不参与预测）
  tf2::Transform last_odom_base_;
  bool have_last_odom_base_{false};

  // ROS 接口
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_clear_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarLoc>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
