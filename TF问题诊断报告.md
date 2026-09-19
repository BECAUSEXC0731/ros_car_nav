# TF 变换问题诊断报告

> 日期：2026-07-30
>
> ⚠️ 后记（2026-09-19）：本文提到的串口雷达驱动 `lidar_node`（`src/lidar_pkg`）已删除，
> `/scan` 改由下位机（STC）直接发布；本文仅作为历史排查记录保留。

---

## 1. 当前 TF 树

```
odom
 └─ base_footprint   ← 里程计 (20 Hz)
      └─ base_link   ← 静态 (URDF)
           ├─ front_marker
           ├─ laser_frame       ← 激光雷达 scan 发布用的 frame_id
           ├─ laser_link
           ├─ left_front_wheel
           ├─ left_rear_wheel
           ├─ right_front_wheel
           └─ right_rear_wheel
```

**缺少的变换：`map` → `odom`**

---

## 2. 问题根因（两个问题叠加）

### 问题一：`map` 帧不存在于 TF 树中

当前 `slam_toolbox` 虽然已启动，**但没有发布 `map` → `odom` 的 TF 变换**。

在 RViz 中将固定坐标系设为 `map` 时，由于 `map` 帧不存在，TF 树找不到从 `map` 到小车模型的路径，导致：

- 小车模型消失 / 显示异常
- 出现 `Invalid frame ID "map" passed to canTransform` 错误

### 问题二：雷达没有在发布数据（更根本的原因）

`/scan` 话题存在，但 **没有收到任何激光雷达数据**：

| 检查项 | 结果 |
|--------|------|
| `/scan` publisher | 有（lidar_node） |
| `/scan` subscriber | 有（slam_toolbox） |
| `/scan` 消息数 | **0 条** |
| 串口设备 `/dev/ttyACM0` | 存在，权限正常 |

`slam_toolbox` 需要激光数据才能建图和计算 `map` → `odom` 变换。没有数据时它只能空转。

---

## 3. 为什么雷达没数据

可能原因：

| 原因 | 可能性 | 说明 |
|------|--------|------|
| 雷达硬件未通电 / 未连接 | 中 | 检查雷达电源线和 USB 是否插好 |
| 串口被占用 | 低 | 检查是否有其他进程占用 `/dev/ttyACM0` |
| 雷达协议不兼容 | 低 | 当前使用的协议是 AA 55 格式 |
| 波特率不匹配 | 低 | 当前配置 150000 |

---

## 4. 修复步骤

### 第一步：确认雷达硬件

```bash
# 检查串口设备
ls -la /dev/ttyACM0

# 检查是否有数据从串口来
cat /dev/ttyACM0 | xxd | head -20
```

如果 `cat` 能看到 `AA 55` 之类的十六进制数据，说明雷达在工作。

### 第二步：重启雷达节点

```bash
# 先找到 lidar_node 进程
ps aux | grep lidar_node

# 重启 launch 文件
# 先 Ctrl+C 停掉当前启动，再重新启动
```

### 第三步：验证雷达数据

```bash
# 确认 /scan 有数据
ros2 topic hz /scan
ros2 topic echo /scan --once
```

### 第四步：检查 `map` → `odom` 是否恢复

```bash
# 检查 map 帧是否存在
ros2 run tf2_ros tf2_echo map base_footprint
```

---

## 5. 其他潜在问题

### slam_toolbox 配置建议

当前配置只有 4 个参数。建议创建专门的配置文件 `slam_params.yaml`：

```yaml
slam_toolbox:
  ros__parameters:
    # 必要参数
    use_sim_time: false
    mode: mapping

    # 坐标系
    base_frame: base_footprint
    odom_frame: odom
    map_frame: map

    # 雷达话题
    scan_topic: /scan

    # 建图参数
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 8.0
    minimum_time_interval: 0.5
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    transform_publish_period: 0.05

    # 闭环
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0

    # 匹配器
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None
```

> 后续进展（2026-09-19）：这个 `slam_params.yaml` 已经落地为
> `src/slam_pkg/config/slam_params.yaml`，并且是**全工程唯一一份** slam 参数。

### 雷达节点改进建议

当前 `lidar_node` 如果串口读不到数据，没有任何重试机制。可以考虑：

1. 添加串口重连逻辑
2. 添加健康检测定时器
3. 添加诊断话题输出

> 后续进展（2026-09-19）：雷达驱动已整体移除，改由下位机（STC）发布 `/scan`，
> 这些问题不再存在；上位机侧改用 `xcar_bringup/launch/robot_base.launch.py` 统一启动。

---

## 6. 快速验证命令汇总

```bash
# 检查 TF 树（生成 PDF/图片）
ros2 run tf2_tools view_frames

# 检查 map 是否存在
ros2 run tf2_ros tf2_echo map base_footprint

# 检查雷达数据
ros2 topic hz /scan
ros2 topic echo /scan --once

# 检查所有话题
ros2 topic list

# 检查所有节点
ros2 node list

# 查看参数
ros2 param dump /slam_toolbox
```
