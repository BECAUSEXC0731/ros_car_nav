# xcar_nav —— Nav2 自主导航包
ros2 run nav2_map_server map_saver_cli -f ~/roscar_nav_last/src/xcar_nav/maps/square

在**已保存的地图**上做自主导航：AMCL 定位 + 全局/局部路径规划 + 速度控制。

## 1. 数据流与 TF 权责

```
/scan ─┐
       ├─→ AMCL ──→ TF: map -> odom          （定位，纠正累积误差）
/odom ─┘                  ↑
                          │ EKF（xcar_bringup）
                     TF: odom -> base_footprint
                          │ URDF（xcar_description）
                     TF: base_footprint -> base_link -> 雷达/IMU

Nav2（planner / controller / behaviors / bt_navigator）
   → /cmd_vel_nav → velocity_smoother → /cmd_vel → 下位机执行
```

> ⚠️ **不要和 `slam_pkg` 的建图同时运行**：建图时 `map -> odom` 由 slam_toolbox 发布，
> 导航时由 AMCL 发布，两个一起跑会互相抢发这段 TF。

## 2. 依赖

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## 3. 使用

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
#    - 用 "2D Pose Estimate" 点一下小车当前在地图上的位置和朝向（初始化 AMCL）
#    - 用 "2D Nav Goal"（Humble 里叫 "2D Goal Pose"）点目标点，小车就会自己过去
```

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `map` | `maps/qinshi.yaml` | 换地图时传入绝对路径 |
| `params_file` | `config/nav2_params.yaml` | Nav2 参数 |
| `use_rviz` | `true` | 无显示器时设 `false` |
| `use_composition` | `false` | `true` 更省内存；`false` 每节点独立进程，便于排查 |
| `nav2_delay` | `3.0` | 等机器人本体 TF 就绪再启动 Nav2 的秒数 |

示例：

```bash
ros2 launch xcar_nav navigation.launch.py use_rviz:=false
ros2 launch xcar_nav navigation.launch.py map:=$HOME/maps/other.yaml
```

## 4. 建新地图 & 更新地图

```bash
# 建图（另开终端，用键盘遥控跑一圈）
ros2 launch slam_pkg slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 存图（直接存进本包）
ros2 run nav2_map_server map_saver_cli -f src/xcar_nav/maps/qinshi
colcon build --packages-select xcar_nav
```

## 5. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| AMCL 不动、粒子不收敛 | 忘了点 `2D Pose Estimate`，或初始朝向点反了 |
| 车不动，但 RViz 里有路径 | 检查 `/cmd_vel` 有没有数据：`ros2 topic echo /cmd_vel`；再看下位机是否订阅同一话题名 |
| 一直报 TF 超时 | 先确认 `map->odom->base_footprint` 三段都存在：`ros2 run tf2_tools view_frames` |
| 规划失败 / 无路径 | 全局代价地图里起点被障碍物包住（膨胀半径太大），或地图里该区域是未知(灰色)，可把 `allow_unknown` 打开（默认已开） |
| 车走起来抖 / 画龙 | 调小 `max_vel_x`、调大 `inflation_radius`，或先用 `teleop` 确认底盘本身速度标定准确 |

## 6. 调参入口

- 机器人尺寸：`footprint`（当前按车体 0.30m × 0.18m 写的矩形）
- 速度上限：`FollowPath.max_vel_x / max_vel_theta`，且必须同步改
  `velocity_smoother.max_velocity`（更小者生效）
- 到点精度：`general_goal_checker.xy_goal_tolerance / yaw_goal_tolerance`
- 安全距离：`local_costmap`/`global_costmap` 的 `inflation_radius`
