# xcar_bringup —— 机器人本体启动包

把下位机的原始数据整理成"能用"的 TF 与话题：**URDF 静态 TF + 里程计/IMU 融合 +
激光时间戳/去畸变处理**。建图和导航都会 include 它。

## 1. 文件说明

| 文件 | 说明 |
|---|---|
| `launch/robot_base.launch.py` | 本体启动入口（下面所有节点） |
| `launch/urdf2tf.launch.py` | 启动 `robot_state_publisher`，发布 URDF 静态 TF |
| `config/ekf.yaml` | `robot_localization` 参数 |
| `src/odom2tf.cpp` | 降级方案：`/odom` 直接转 TF（`use_ekf:=false` 时用） |
| `scripts/imu_repub.py` | `/imu` → `/imu_rotated`（frame 改名、协方差兜底、陀螺零偏估计） |
| `scripts/odom_cov_repub.py` | `/odom` → `/odom_cov`（只改协方差，降低 yaw 权重） |
| `scripts/scan_repub.py` | `/scan` → `/scan_ts`（按上位机时钟重打戳，对齐扫描中点） |
| `scripts/scan_deskew.py` | `/scan_ts` → `/scan_cloud`（转镜雷达运动畸变补偿，PointCloud2） |

## 2. 启动

```bash
ros2 launch xcar_bringup robot_base.launch.py               # 默认：用 EKF
ros2 launch xcar_bringup robot_base.launch.py use_ekf:=false # 降级：odom2tf 直通
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_ekf` | `true` | `true` = EKF 融合；`false` = 只把 `/odom` 转成 TF（无 IMU 时用） |

单独看模型：`ros2 launch xcar_bringup urdf2tf.launch.py`（或 `xcar_description/display.launch.py`）。

## 3. 节点与数据流

```
/scan ──→ scan_repub ──→ /scan_ts ──→ scan_deskew ──→ /scan_cloud (PointCloud2, 激光系)
/odom ──→ odom_cov_repub ──→ /odom_cov ─┐
/imu  ──→ imu_repub     ──→ /imu_rotated ┴─→ ekf_node ──→ TF: odom -> base_footprint
                                                          /odom/filtered
xcar.urdf ──→ robot_state_publisher ──→ TF: base_footprint -> base_link -> 雷达/IMU/轮子
```

| 节点 | 输入 | 输出 | 关键参数 |
|---|---|---|---|
| `robot_state_publisher` | `urdf/xcar.urdf` | 静态 TF | `publish_frequency: 10` |
| `imu_repub` | `/imu` | `/imu_rotated` | `frame_id`, `estimate_gyro_bias` |
| `odom_cov_repub` | `/odom` | `/odom_cov` | `x/y_variance`, `yaw_variance` |
| `scan_repub` | `/scan` | `/scan_ts` | `restamp: True`, `reliable: True`, `compensate_scan_time` |
| `scan_deskew` | `/scan_ts` | `/scan_cloud` | `world_frame: odom`, `robot_frame: base_footprint` |
| `ekf_node` | `/odom_cov`, `/imu_rotated` | TF `odom->base_footprint` | `config/ekf.yaml` |
| `odom2tf` | `/odom` | TF `odom->base_footprint` | `restamp: True`（仅 `use_ekf:=false`） |

`/odom` 本身保持原样，Nav2 等照常订阅；`/scan` 原始话题也保留。

## 4. 为什么要有这些节点（设计说明）

1. **时钟对齐**：下位机 `/scan` 的时间戳比上位机旧 0.4~0.7 s，会让 tf2 大量报
   `extrapolation` / `Transform data too old`。`scan_repub` 按接收时刻重打戳；
   转镜雷达还要再减 `scan_time/2`，把时间戳对齐到**扫描中点**。
2. **运动畸变**：5 Hz 转镜雷达一圈 200 ms，224 束点各自采样时刻不同。车旋转时
   用单一时间戳变换整圈点，会把墙"糊"开。`scan_deskew` 按每束点真实时刻插值补偿，
   输出保持在**激光系**（否则 obstacle_layer 的射线清除原点会错）。
3. **EKF 只信该信的**：轮式里程计 x/y 长期稳定（信），yaw 实测方向反 + 比例约 48%
   （**不信**，`odom0_config` 里 yaw=false）；航向由 IMU 的 `vyaw` 积分主导，
   绝对位姿由 `lidar_loc` 扫描匹配纠正。
4. **零偏**：`imu_repub` 在"已指令停止且 vyaw 足够小"时滑动平均估计陀螺零偏并扣除，
   估计值周期性打进日志。

## 5. 验证

```bash
ros2 topic hz /scan_ts        # ≈5 Hz
ros2 topic hz /scan_cloud     # ≈5 Hz（没数据先看 scan_deskew 的 WARN 日志）
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_monitor odom base_footprint   # Average Delay 应为几十 ms
ros2 topic echo /odom/filtered --once
```

## 6. 注意

- Python 脚本首行必须是 `#!/usr/bin/python3`（Humble 的 `rclpy` 只认系统 py3.10）。
- 新增脚本要加进 `CMakeLists.txt` 的 `install(PROGRAMS)`。
- `odom -> base_footprint` **只能有一个发布者**：EKF 或 `odom2tf`，不要同时开。
- `scan_deskew` 查 TF 时用 `Time()`（零时刻 = 最新可用），不要查 `now()`，
  否则会因"未来时刻"抛外推异常并整帧丢弃。
