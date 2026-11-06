# xcar_jiguang —— 纯激光地图匹配定位（替代 AMCL）

从 ROS1 的 `jie_ware/lidar_loc` 移植到 ROS2 Humble，并改名/重写为 `xcar_jiguang`。
作用与 AMCL 相同：**订阅静态地图 + 激光，发布 `map -> odom` TF**，让 Nav2 拿到全局位姿。
差别是：不用粒子滤波、不用里程计做预测，直接把每一帧激光和地图做匹配。

```
/map       ──┐
              ├─→ lidar_loc ──→ TF: map -> odom
/scan_cloud ──┘         └──────→ /amcl_pose（位姿，RViz 显示用）
  ↑
  scan_deskew（去畸变点云，激光系；use_deskewed_scan:=false 时改订阅 /scan_ts）
```

---

## 1. 算法

1. **预处理地图**（收到 `/map` 时做一次）
   - 提取占据格子，裁到障碍物包围盒（外扩 `gradient_radius`），省内存。
   - 用 `cv::distanceTransform` 算"每个像素到最近障碍物的距离 d"，
     生成**高斯似然场** `field = 255 · exp(-d²/2σ²)`（σ = `field_sigma`，默认 0.3 m），
     距离超过 `gradient_radius`（默认 1.0 m）记 0。
2. **每帧激光**
   - 用 TF 把激光点从 `laser_frame` 变到 `base_frame`（自动处理雷达安装位置/朝向/倒装）。
   - **先用里程计把上一帧位姿推到当前时刻**当搜索起点：
     `T_map_base(new) = T_map_base(old) · (T_odom_base(old))⁻¹ · T_odom_base(new)`
   - 爬山搜索位姿 `(x, y, yaw)`：每次在 5 个平移候选 × 3 个朝向候选里挑让
     "所有激光点在似然场里的取值之和"最大的那个，直到 15 个候选都不再改善。
     分两轮：粗搜（5 cm / 1°）→ 精修（1 cm / 0.2°）。场值用**双线性插值**采样，
     否则目标函数是 5 cm 一级的台阶，静止时位姿会在相邻格之间来回跳。
   - 得分归一化成**质量** `q = 得分 / (255 × 点数)`，q≈1 表示所有点都贴在地图墙线上。
   - **平滑 + 死区**：修正量小于 `deadband_trans`(2 cm) / `deadband_yaw`(0.5°) 时直接
     沿用里程计预测位姿（静止时零抖动）；否则只采纳 `smoothing_alpha`（默认 0.5）倍的修正。
3. **输出**
   - 30 Hz 用 `T_map_odom = T_map_base · (T_odom_base)⁻¹` 发布 `map -> odom`（时间戳取当前时刻，
     不是 AMCL 那种 `scan_stamp + transform_tolerance`，所以没有位姿滞后）。
   - 每帧激光发一次 `/amcl_pose`（保留 AMCL 的话题名，RViz/外部脚本不用改）。

为什么快：渐进搜索只在上一帧位姿附近做局部爬山，一帧通常 10~60 次迭代、几分钟内跑满 10 Hz。

---

## 2. 话题 / TF / 参数

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/map` | `nav_msgs/OccupancyGrid` | transient_local QoS，`map_server` 发布 |
| 订阅 | `/scan_cloud` | `sensor_msgs/PointCloud2` | **默认**输入：`scan_deskew` 输出的去畸变点云（激光系） |
| 订阅 | `/scan_ts` | `sensor_msgs/LaserScan` | 回退输入（`use_deskewed_scan: False`），由 `scan_repub` 重打戳 |
| 订阅 | `/initialpose` | `PoseWithCovarianceStamped` | RViz 的 "2D Pose Estimate" |
| 发布 | `/amcl_pose` | `PoseWithCovarianceStamped` | 当前位姿 |
| 发布 | `/tf` | `map -> odom` | 定位结果 |
| 调用 | `/local_costmap/clear_entirely_local_costmap`<br>`/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | 重定位/静止 3 s 后清空一次代价地图 |

参数见 `config/lidar_loc.yaml`，最常调的：

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_deskewed_scan` | `True` | 用 `/scan_cloud` 匹配（移动/旋转时抖动更小）；`False` 回退 `/scan_ts` |
| `scan_cloud_topic` | `scan_cloud` | 去畸变点云话题 |
| `scan_topic` | `scan_ts` | 回退用的激光话题（换回原始数据就改 `scan`） |
| `laser_frame` | `laser_frame` | 留空 `""` 则用 `scan.header.frame_id` |
| `set_initial_pose` | `True` | 用 `initial_pose` 作初值；`False` 则等 RViz 点选 |
| `initial_pose.{x,y,yaw}` | 0 | 车辆每次停放位置不同时，建议设 `False` 并手工点选 |
| `field_sigma` | 0.30 | 似然场标准差(米)。**越小越精确/抗抖，太小容易跟丢** |
| `gradient_radius` | 1.0 | 距离超过该值记 0 分(米)。旧版用 2.5m 线性渐变，得分面太平→抖动 |
| `search_step_ratio` | 1.0 | 粗搜步长 = 分辨率 × 该值（1.0 → 5 cm/步） |
| `yaw_step_deg` | 1.0 | 粗搜朝向步长(度) |
| `refine_ratio` | 0.2 | 精修步长比例（→ 1 cm / 0.2°），消除量化抖动 |
| `min_quality` | 0.15 | 质量门槛，低于它丢弃该帧（保持上次位姿） |
| `min_points` | 40 | 本帧有效激光点少于它直接忽略（防垃圾帧把位姿带飞） |
| `max_correction_trans/rot` | 0.5 / 0.4 | 单帧修正量上限（相对里程计预测），超过判为发散 |
| `smoothing_alpha` | 0.5 | 每次只采纳该比例的修正；`1.0` = 不平滑（更抖但跟得更紧） |
| `deadband_trans` / `deadband_yaw_deg` | 0.02 / 0.5 | 修正量死区：小于它就完全沿用里程计预测（静止时零抖动） |
| `global_relocalize` | `True` | 没有初值时做一次全局粗搜索重定位 |
| `global_candidates` | 5 | 粗搜索后复核的候选数（对抗对称房间的 180° 镜像解） |

---

## 3. 使用

正常导航时由 `xcar_nav/navigation.launch.py` 自动拉起，不用单独启动。

```bash
cd ~/roscar_nav_last
colcon build --packages-select xcar_jiguang
source install/setup.bash
ros2 launch xcar_nav navigation.launch.py
```

前置条件：`/map`（map_server）、`odom -> base_footprint` TF（EKF / odom2tf）、
输入激光（默认 `/scan_cloud`，由 `xcar_bringup` 的 `scan_deskew` 发布）。

只调试定位（自己起 map_server + RViz）：

```bash
ros2 launch xcar_jiguang lidar_loc.launch.py                 # 默认地图 maps/shiyan.yaml
# 另开终端确认 TF：
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /scan_cloud        # 没数据时先看 scan_deskew 的 WARN 日志
```

想让车在地图上"原地重定位"：RViz 里点 **2D Pose Estimate** → 发布 `/initialpose` →
节点立刻把位姿切过去，之后继续跟踪。

---

## 4. 怎么看它工作得好不好

日志每 `log_period`(默认 5s) 打印一行：

```
位姿 x=2.150 y=-0.870 yaw=-1.3° | 质量 0.993 | 本帧修正 0.8 cm / 0.12° | 360 点
```

- **质量稳定在 0.9 以上** → 匹配很贴合，正常（高斯场 σ=0.3 下，0.9 约等于平均偏 8 cm）。
- **“本帧修正”始终很小（<2 cm / 0.5°）** → 处于死区内，位姿直接跟随里程计，静止时零抖动。
- **质量忽高忽低** → 激光噪点/动态障碍太多，适当加大 `field_sigma`。
- **`匹配修正量过大（x.xx m），已丢弃本帧`** → 位姿在跳，先点一次 2D Pose Estimate。
- **`匹配质量过低（0.0xx < 0.15）`** → 当前位姿和地图不符（可能被搬动了、或地图不是这一张）。
- **`本帧只有 N 个有效激光点（< 40），忽略`** → 雷达数据异常（大量 inf/近距离遮挡），已丢弃。
- **`全局重定位存在歧义`** → 这张地图近似对称，粗搜索分不清正反，请手工给初值。

---

## 5. 相对原版 `jie_ware/lidar_loc`（ROS1）改了什么

| 项目 | 原版 | 现版本 |
|---|---|---|
| 构建 | catkin / `ros::NodeHandle` | ament_cmake / `rclcpp`，可在 ROS2 Humble 工作空间编译 |
| **几何一致性** | 激光点先取反 y，再用 `lidar_y - rotated_y` 匹配，等价于把"旋转引起的位置耦合"镜像了一次；yaw≈0 时对，旋转后越走越偏 | 统一为标准刚体变换 `x_map = c·x − s·y + X`、`y_map = s·x + c·y + Y` |
| 渐变场 | 每个障碍物贴 101×101 掩模取 max，O(障碍数·101²) | 一次 `distanceTransform`，O(W·H)，同效果但快一个量级 |
| 收敛 | `while(ros::ok())`，靠 10 帧位移阈值退出，理论上可能长时间不退出 | 15 个候选都不改善即收敛，另有 `max_iterations` 硬上限 |
| 健壮性 | 无 | 质量门槛、单帧修正量上限、地图重复接收跳过重建、TF 查不到只告警 |
| 重定位 | 收到地图时把位姿重置到 map 原点 (0,0)，靠爬山慢慢挪 | 粗搜索（0.3 m × 15°）+ Top-K 精细复核 + 歧义告警 |
| 代价地图清理 | 调 `move_base/clear_costmaps` | 去掉（Nav2 没有该服务，代价地图靠 raytrace 自行清除） |
| 其它 | `costmap_cleaner`、`lidar_filter_node`、Gazebo launch | 未移植（本车用不到），只保留 `lidar_loc` |

---

## 6. 已知限制（务必了解）

这是**局部**匹配器，不是 AMCL 那种全局概率定位：

- **需要还算靠谱的初值。** 初值和真值差 >1~2 m 或 >30° 时，爬山容易停在错误的局部最优。
  所以推荐每次启动由人点一次 `2D Pose Estimate`（或把 `initial_pose` 写成实车常用位置）。
- **对称/长走廊环境会歧义**（180° 镜像、沿墙平移得分几乎相同）。节点会打歧义告警，
  但最终仍需人工确认。
- **没有粒子、没有恢复行为。** 车被搬走或长时间被遮挡后会跟丢，只能重新点 2D Pose Estimate。
- **动态障碍物会拉偏得分。** 场地里人或箱子多时，适当减小 `gradient_radius`。

如果实车表现不理想，随时可以退回 AMCL：把 `src/xcar_nav/launch/navigation.launch.py`
恢复成 `nav2_bringup/bringup_launch.py`（`slam:=False`）并在 `nav2_params.yaml` 里补回
`amcl:` 段即可（历史上那份配置见 git 记录）。
