# tools —— 离线验证 / 实车诊断脚本

这些脚本是 2026-10-28 做「纯激光定位替代 AMCL」和「抖动/噪点排查」时写的，
全部用系统 Python（`/usr/bin/python3`，Humble 的 rclpy 只在 py3.10 下可用），
**不要用 conda 的 python**。

| 脚本 | 用途 |
|---|---|
| `lidar_sim.py` | 用 pgm 地图做射线投射，发布假 `/map` + `/laser_sim`，并让"真值"按轨迹运动（静止 → 转 0.23 rad → 前进 0.6 m）。参数：`<map.yaml>` |
| `lidar_loc_sim_test.yaml` | 配合 `lidar_sim.py` 的定位参数（订阅 `laser_sim`、给定近似初值） |
| `jitter.py` | 统计 `/amcl_pose` 稳态抖动。用法：`jitter.py <总时长s> <跳过前Ns>` |
| `diag.py` | 实车诊断：`/scan` 数据质量（有效/非有限/孤立噪点数）+ `/amcl_pose` 逐帧变化 |
| `costmap_noise.py` | 统计代价地图里的孤立噪点。用法：`costmap_noise.py /local_costmap/costmap` |
| `check_tf.py` | 等 15 s 打印 `map->odom` 的平移与 yaw，确认 TF 在发 |
| `replica.py` | 纯 numpy 复现 lidar_loc 的评分/爬山逻辑，用于排查 C++ 与算法推导是否一致 |
| `dump_costmap.py` | 把代价地图打成 ASCII 图 + 连通块统计。用法：`dump_costmap.py /local_costmap/costmap` |
| `explain_costmap.py` | 判断致命格是“真实障碍”还是“残留”：用当前激光点云去解释代价地图 |
| `blob_check.py` | 把局部代价地图致命格变换到 map 系，统计距静态墙线的距离分布 |
| `scan_age.py` | 搜索激光数据的“真实时间延迟”（用静态地图做基准对比对齐误差） |
| `measure.py` | 实测 `/scan` `/scan_ts` 频率与时间戳年龄 + `/amcl_pose` 抖动 |

## 典型离线验证流程

```bash
cd ~/roscar_nav_last && source /opt/ros/humble/setup.bash && source install/setup.bash

# 1) 假 TF（真车上由 URDF/EKF 提供）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint laser_frame &

# 2) 合成激光 + 地图
/usr/bin/python3 tools/lidar_sim.py src/xcar_nav/maps/shiyan.yaml &

# 3) 定位节点（参数里 scan_topic=laser_sim）
ros2 run xcar_jiguang lidar_loc --ros-args \
  --params-file tools/lidar_loc_sim_test.yaml

# 4) 看结果
ros2 run tf2_ros tf2_echo map odom
/usr/bin/python3 tools/jitter.py 20 6
```

`lidar_sim.py` 里 `raycast(x, y, yaw + angle)` 的 yaw 不能漏——
漏了的话机器人"旋转"时激光数据不变，会误以为定位不跟转向（踩过一次）。
