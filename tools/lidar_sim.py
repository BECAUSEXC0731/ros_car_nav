#!/usr/bin/python3
"""离线验证 xcar_jiguang/lidar_loc：用 pgm 地图做射线投射生成合成 LaserScan。

- 发布 /map（transient_local）
- 按"真值位姿" raycast 出 360 束激光，发布 /laser_sim（frame_id=laser_frame, 1°/束）
- 真值位姿按预设轨迹运动：静止 → 原地转 0.4rad → 前进 0.6m
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, idx = [], 0
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] != b'\n':
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    idx += 1
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)


def main():
    import yaml as pyyaml

    yaml_path = sys.argv[1]
    meta = pyyaml.safe_load(open(yaml_path))
    img = read_pgm(str(yaml_path).rsplit('/', 1)[0] + '/' + meta['image'])
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    occ = img < 100
    unknown = (img >= 200) & (img < 250)

    # OccupancyGrid: 索引 = y*w+x，row 0 对应地图 y 最小处 → pgm 垂直翻转
    occ_f = occ[::-1, :]
    unk_f = unknown[::-1, :]
    h, w = occ_f.shape
    grid = np.full((h, w), -1, dtype=np.int8)
    grid[~occ_f & ~unk_f] = 0
    grid[occ_f] = 100

    rclpy.init()
    node = Node('lidar_sim')

    map_qos = QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    grid_msg = OccupancyGrid()
    grid_msg.header.frame_id = 'map'
    grid_msg.info.resolution = res
    grid_msg.info.width = w
    grid_msg.info.height = h
    grid_msg.info.origin.position.x = ox
    grid_msg.info.origin.position.y = oy
    grid_msg.info.origin.orientation.w = 1.0
    grid_msg.data = grid.reshape(-1).tolist()

    map_pub = node.create_publisher(OccupancyGrid, '/map', map_qos)
    scan_pub = node.create_publisher(LaserScan, '/laser_sim', 10)
    map_pub.publish(grid_msg)

    # 用上下扫描的近似距离场挑一个最空旷的空闲格当起点
    free_mask = grid == 0
    BIG = 1e6
    up = np.full((h, w), BIG, dtype=np.float32)
    for r in range(h):
        prev = up[r - 1] if r > 0 else np.full(w, BIG, dtype=np.float32) + 1
        up[r] = np.where(free_mask[r], np.minimum(prev + 1, BIG), 0)
    down = np.full((h, w), BIG, dtype=np.float32)
    for r in range(h - 1, -1, -1):
        nxt = down[r + 1] if r < h - 1 else np.full(w, BIG, dtype=np.float32) + 1
        down[r] = np.where(free_mask[r], np.minimum(nxt + 1, BIG), 0)
    clearance = np.minimum(up, down) * res
    clearance[~free_mask] = 0
    r0, c0 = np.unravel_index(int(np.argmax(clearance)), clearance.shape)
    truth_x = ox + (c0 + 0.5) * res
    truth_y = oy + (r0 + 0.5) * res
    node.get_logger().info(
        f'真值起点 x={truth_x:.2f} y={truth_y:.2f}，净空 {clearance[r0, c0]:.2f} m；'
        f'地图 {w}x{h} @ {res} origin=({ox},{oy})')

    start = node.get_clock().now()

    def raycast(x, y, a):
        dx, dy = math.cos(a), math.sin(a)
        step = res * 0.5
        rng = 0.0
        while rng < 8.0:
            c = int((x + dx * rng - ox) / res)
            r = int((y + dy * rng - oy) / res)
            if not (0 <= r < h and 0 <= c < w) or grid[r, c] != 0:
                return rng
            rng += step
        return 8.0

    scan = LaserScan()
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -math.pi
    scan.angle_increment = math.radians(1)
    n = 360
    scan.angle_max = scan.angle_min + (n - 1) * scan.angle_increment
    scan.range_min = 0.05
    scan.range_max = 8.0
    scan.ranges = [8.0] * n

    last_log = [-10.0]

    def tick():
        el = (node.get_clock().now() - start).nanoseconds * 1e-9
        yaw = 0.0
        x, y = truth_x, truth_y
        if el > 6.0:
            yaw = min(0.4, (el - 6.0) * 0.4)
        if el > 12.0:
            d = min(0.6, (el - 12.0) * 0.1)
            x += math.cos(yaw) * d
            y += math.sin(yaw) * d
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.ranges = [raycast(x, y, yaw + scan.angle_min + i * scan.angle_increment)
                       for i in range(n)]
        scan_pub.publish(scan)
        if el - last_log[0] >= 2.0:
            last_log[0] = el
            node.get_logger().info(
                f'[t={el:4.1f}s] 真值 x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):6.1f}°')

    node.create_timer(0.1, tick)
    node.create_timer(2.0, lambda: map_pub.publish(grid_msg))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
