# slam_pkg —— slam_toolbox 建图

用下位机发布的 `/scan` 建 2D 栅格地图，输出 `/map` 与 `map -> odom` TF。
建完图用 `map_saver_cli` 存成 pgm/yaml，给 `xcar_nav` 导航用。

## 1. 文件说明

| 文件 | 说明 |
|---|---|
| `launch/slam_robot.launch.py` | 唯一入口：机器人本体 + slam_toolbox（+ 可选 RViz） |
| `config/slam_params.yaml` | slam_toolbox 参数（全工程唯一一份） |
| `CMakeLists.txt` | 只安装 `launch/` 和 `config/`，无编译代码 |

## 2. 启动

```bash
cd ~/roscar_nav_last
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch slam_pkg slam_robot.launch.py                 # 无 RViz
ros2 launch slam_pkg slam_robot.launch.py use_rviz:=true  # 带 RViz（xcar_description/display.rviz）
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_rviz` | `false` | 是否同时启动 RViz2 |

## 3. 组成

```
robot_base.launch.py   ← 机器人本体（URDF 静态 TF + EKF / odom2tf）
    ├─ robot_state_publisher （xcar.urdf）
    └─ ekf_node 或 odom2tf   → TF: odom -> base_footprint

slam_toolbox (sync_slam_toolbox_node)
    /scan → /map, TF: map -> odom
```

本 launch **不启动** `odom2tf`、AMCL、Nav2，避免抢发 TF。

## 4. 建图流程

```bash
# 1) 起建图
ros2 launch slam_pkg slam_robot.launch.py use_rviz:=true

# 2) 另开终端遥控走一圈（慢一点，转弯别太快）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3) 存图（直接存进导航包）
ros2 run nav2_map_server map_saver_cli -f src/xcar_nav/maps/新地图名
colcon build --packages-select xcar_nav
```

## 5. 参数要点（`config/slam_params.yaml`）

针对"小场地 + 小底盘 + 慢速"调过，阈值比 slam_toolbox 默认值小很多：

| 参数 | 值 | 说明 |
|---|---|---|
| `scan_topic` | `/scan` | 直接用原始激光（建图不要求低时延） |
| `map_update_interval` | `0.5` | 地图刷新间隔 [s]（默认 5.0） |
| `resolution` | `0.05` | 栅格分辨率 [m]，与导航地图一致 |
| `minimum_travel_distance` | `0.1` | 走 10 cm 就加一个节点（默认 0.5） |
| `minimum_travel_heading` | `0.05` | 转 2.9° 就加一个节点 |
| `transform_publish_period` | `0.05` | `map -> odom` 发布周期 [s] |
| `max_laser_range` | `8.0` | 与 `/scan` 的 `range_max` 对齐 |
| `solver_plugin` | `CeresSolver` | 扫描匹配求解器 |

## 6. 注意

- **不要和 `xcar_nav navigation.launch.py` 同时运行**，两者都发 `map -> odom`。
- 建图质量差（墙重影、闭环错）时：先查 TF（`tf2_monitor`），再看 `/scan_ts`
  时间戳是否滞后；转镜雷达的运动畸变在快速旋转时最明显，慢速走更稳。
- 存图后的 pgm 若覆盖了正在用的地图，需 `colcon build --packages-select xcar_nav`
  并重启导航。
