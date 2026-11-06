# xcar_description —— 机器人 URDF 模型

小车几何与坐标系描述（底盘、4 个轮子、激光雷达、IMU），以及配套 RViz 配置。
由 `xcar_bringup` 的 `robot_state_publisher` 加载，生成静态 TF。

## 1. 文件说明

| 文件 | 说明 |
|---|---|
| `urdf/xcar.urdf` | 机器人模型（全部为 `fixed` joint） |
| `launch/display.launch.py` | 只显示模型 + RViz，不依赖底盘数据 |
| `rviz/display.rviz` | 模型显示配置 |

## 2. 坐标系

```
base_footprint          地面投影（URDF 根，与 EKF 的 base_link_frame 一致）
└── base_joint (fixed)
    └── base_link       车体
        ├── front_marker                车头方向标记
        ├── left/right_front/rear_wheel 四个轮子
        ├── imu_joint → imu_link        MPU 安装位（rpy z = -0.6109 = -35°）
        ├── laser_joint → laser_link    雷达（rpy z = -1.2217 = -70°）
        └── laser_frame_joint → laser_frame  （rpy z = 2.4435 = 140°）
```

> `/scan` 的 `frame_id` 是 **`laser_frame`**，不是 `laser_link`。两套雷达 frame
> 是为了分别适配"雷达原始朝向"和"修正后的朝向"；改 URDF 时注意别只改一个。

## 3. 使用

```bash
ros2 launch xcar_description display.launch.py
# 自定义模型路径：
ros2 launch xcar_description display.launch.py model:=/abs/path/to/robot.urdf
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `model` | `<share>/urdf/xcar.urdf` | URDF 绝对路径 |

本 launch 自己启动 `robot_state_publisher`，不 include `xcar_bringup`，避免包间循环依赖。

## 4. 修改指引

- **雷达/IMU 安装角度**：改对应 joint 的 `origin rpy`（弧度）。每 5° = 0.0873 rad。
  改完必须重启 `robot_base.launch.py` 或 `display.launch.py`（URDF 只在启动时读取）。
- **车体尺寸**：`base_link` 的 `<box size>`；同时要同步 `xcar_nav/config/nav2_params.yaml`
  里的 `footprint`（矩形四角），否则避障会不准。
- 所有 joint 都是 `fixed`，**没有** `joint_state_publisher`，也不需要
  `/joint_states`。
- 后续若加云台/舵机等可动关节，需把该 joint 改为 `revolute/continuous` 并补上
  `joint_state_publisher`。
