# xcar_nav —— Nav2 自主导航包

在**已保存的地图**上做自主导航：纯激光地图匹配定位（`xcar_jiguang`，不用 AMCL）+
全局/局部路径规划 + 速度控制。

## 1. 文件说明

| 文件 | 说明 |
|---|---|
| `launch/navigation.launch.py` | 唯一入口：本体 + map_server + lidar_loc + Nav2（+ 可选 RViz） |
| `config/nav2_params.yaml` | 全部 Nav2 节点参数 |
| `maps/*.pgm` / `*.yaml` | 静态地图：`shiyan`（默认）、`qinshi`、`square` |
| `rviz/nav2_view.rviz` | 导航用 RViz 配置（激光显示订阅 `/scan_ts`） |

## 2. 数据流与 TF 权责

```
/scan ──→ scan_repub ──→ /scan_ts ─┬─→ scan_deskew ──→ /scan_cloud ─┐
                                   │                                 │
                                   ├─────────────→ global_costmap    │
                                   └─────────────→ lidar_loc ←───────┘
                                                        │
                                                        └─→ TF: map -> odom

/odom ──→ odom_cov_repub ──→ EKF ──→ TF: odom -> base_footprint
/imu  ──→ imu_repub ───────────────↗

URDF（xcar_description）→ TF: base_footprint -> base_link -> 雷达/IMU

Nav2（planner / controller / behaviors / bt_navigator）
   → /cmd_vel_nav → velocity_smoother → /cmd_vel → 下位机执行
```

- `local_costmap` 的障碍层用 `/scan_cloud`（去畸变点云，避免旋转拖影）
- `global_costmap` 的障碍层用 `/scan_ts`（`marking: false`，只用于实时避障显示）
- 激光在 RViz 里显示 `/scan_ts`（原始 `/scan` 时间戳比上位机旧 0.4~0.7 s）

> ⚠️ **不要和 `slam_pkg` 的建图同时运行**：建图时 `map -> odom` 由 slam_toolbox 发布，
> 导航时由 `xcar_jiguang/lidar_loc` 发布，两个一起跑会互相抢发这段 TF。

## 3. 依赖

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## 4. 使用

```bash
# 0) 先启动下位机，确认三个话题都有数据
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu

# 1) 启动导航（含 RViz）
cd ~/roscar_nav_last
source install/setup.bash
ros2 launch xcar_nav navigation.launch.py

# 2) 在 RViz 里：
#    - 用 "2D Pose Estimate" 点一下小车当前在地图上的位置和朝向（给 lidar_loc 一个初值）
#    - 用 "2D Nav Goal"（Humble 里叫 "2D Goal Pose"）点目标点，小车就会自己过去
```

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `map` | `maps/shiyan.yaml` | 换地图时传入绝对路径 |
| `params_file` | `config/nav2_params.yaml` | Nav2 参数 |
| `use_rviz` | `false` | 需要图形界面时设 `true` |
| `use_composition` | `false` | `true` 更省内存；`false` 每节点独立进程，便于排查 |
| `nav2_delay` | `3.0` | 等机器人本体 TF 就绪再启动 Nav2 的秒数 |

示例：

```bash
ros2 launch xcar_nav navigation.launch.py use_rviz:=true
ros2 launch xcar_nav navigation.launch.py map:=$PWD/src/xcar_nav/maps/qinshi.yaml
```

## 5. 建新地图 & 更新地图

已有地图（`maps/`，均为 0.05 m 分辨率）：

| 地图 | 用途 |
|---|---|
| `shiyan.yaml` | 导航默认地图 |
| `qinshi.yaml` | 寝试 / 长走廊场景 |
| `square.yaml` | 方形小场地 |

```bash
# 建图（另开终端，用键盘遥控跑一圈）
ros2 launch slam_pkg slam_robot.launch.py use_rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 存图（直接存进本包）
ros2 run nav2_map_server map_saver_cli -f src/xcar_nav/maps/新地图名
colcon build --packages-select xcar_nav
```

> 建图与导航**不能同时运行**，两者都发布 `map -> odom`。

## 6. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 定位不动 / 位姿乱跳 | 忘了点 `2D Pose Estimate`，或初值差太远；看 `lidar_loc` 日志里的“质量”是否 >0.9 |
| 日志报“全局重定位存在歧义” | 地图近似对称，粗搜索选到了 180° 镜像解，手工点一次 2D Pose Estimate |
| 车不动，但 RViz 里有路径 | 检查 `/cmd_vel` 有没有数据：`ros2 topic echo /cmd_vel`；再看下位机是否订阅同一话题名 |
| 一直报 TF 超时 | 先确认 `map->odom->base_footprint` 三段都存在：`ros2 run tf2_tools view_frames` |
| 规划失败 / 无路径 | 全局代价地图里起点被障碍物包住（膨胀半径太大），或地图里该区域是未知(灰色)，可把 `allow_unknown` 打开（默认已开） |
| 车走起来抖 / 画龙 | 调小 `max_vel_x`、调大 `inflation_radius`，或先用 `teleop` 确认底盘本身速度标定准确 |
| 旋转时局部代价地图“糊”成一片 | 先看 `ros2 topic hz /scan_cloud` 是否 ≈5 Hz（`scan_deskew` 没跑或 TF 查不到会丢帧） |
| 局部地图出现擦不掉的障碍块 | 被墙遮挡的区域没有射线清除（Nav2 固有行为）；停车 3 s 后 `lidar_loc` 会自动清空一次 |

## 7. 调参入口

- 机器人尺寸：`footprint`（当前按车体 0.30m × 0.18m 写的矩形）
- 速度上限：`FollowPath.max_vel_x / max_vel_theta`，且必须同步改
  `velocity_smoother.max_velocity`（更小者生效）
- 到点精度：`general_goal_checker.xy_goal_tolerance / yaw_goal_tolerance`
- 安全距离：`local_costmap`/`global_costmap` 的 `inflation_radius`
