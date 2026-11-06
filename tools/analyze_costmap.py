#!/usr/bin/python3
"""分析 bag 里的代价地图：致命格是否贴墙（被去畸变点云解释）+ 致命格厚度（细线=干净，厚块=拖影）。
用法：/usr/bin/python3 tools/analyze_costmap.py /tmp/smear_test
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from collections import defaultdict

bag = sys.argv[1] if len(sys.argv) > 1 else '/tmp/smear_test'
LASER_YAW = 2.4435  # URDF: base_link -> laser_frame 的 yaw（纯旋转，2D 下无平移）

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))

clouds = []
tfs = []
maps = []
while reader.has_next():
    topic, data, t = reader.read_next()
    ts = t * 1e-9
    if topic == '/scan_cloud':
        m = deserialize_message(data, PointCloud2)
        pts = [(p[0], p[1]) for p in pc2.read_points(m, field_names=('x', 'y'), skip_nans=True)]
        clouds.append((ts, pts))
    elif topic == '/tf':
        for tf in deserialize_message(data, TFMessage).transforms:
            tfs.append((ts, tf))
    elif topic == '/local_costmap/costmap':
        maps.append((ts, deserialize_message(data, OccupancyGrid)))

# odom->base TF 序列
ob = sorted([(ts, tf) for ts, tf in tfs if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint'], key=lambda x: x[0])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ob_at(t):
    if not ob:
        return None
    # 找最近两个做线性插值
    import bisect
    ts_arr = [x[0] for x in ob]
    i = bisect.bisect_left(ts_arr, t)
    if i == 0:
        return ob[0][1]
    if i >= len(ob):
        return ob[-1][1]
    t0, tf0 = ob[i - 1]
    t1, tf1 = ob[i]
    f = (t - t0) / max(1e-9, t1 - t0)
    f = max(0.0, min(1.0, f))
    y0, y1 = yaw_of(tf0.transform.rotation), yaw_of(tf1.transform.rotation)
    y = y0 + math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)) * f
    x = tf0.transform.translation.x + (tf1.transform.translation.x - tf0.transform.translation.x) * f
    yy = tf0.transform.translation.y + (tf1.transform.translation.y - tf0.transform.translation.y) * f
    return x, yy, y


def laser_to_odom(p, ob_t):
    """laser 系点 -> odom 系点：先绕 LASER_YAW 转，再用 odom->base 的 (x,y,yaw) 转"""
    c, s = math.cos(LASER_YAW), math.sin(LASER_YAW)
    xb = c * p[0] - s * p[1]
    yb = s * p[0] + c * p[1]
    if ob_t is None:
        return None
    xo, yo, yo_w = ob_t
    c2, s2 = math.cos(yo_w), math.sin(yo_w)
    return (c2 * xb - s2 * yb + xo, s2 * xb + c2 * yb + yo)


# 取最后几帧代价地图做统计
print('帧数: cloud', len(clouds), 'map', len(maps))
for tm, cmap in maps[-3:]:
    g = np.array(cmap.data, dtype=np.int16).reshape(cmap.info.height, cmap.info.width)
    lethal = g >= 99
    if lethal.sum() == 0:
        print(f'  t={tm:.1f} 致命格 0，跳过')
        continue
    # 最近 cloud
    tc, pts = min(clouds, key=lambda x: abs(x[0] - tm))
    ob_t = ob_at(tc)
    ox, oy, res = cmap.info.origin.position.x, cmap.info.origin.position.y, cmap.info.resolution
    # 栅格化云点
    grid = set()
    for p in pts:
        q = laser_to_odom(p, ob_t)
        if q is None:
            continue
        c = int((q[0] - ox) / res)
        r = int((q[1] - oy) / res)
        for drr in (-1, 0, 1):
            for dcc in (-1, 0, 1):
                grid.add((r + drr, c + dcc))
    total = int(lethal.sum())
    explained = sum(1 for r in range(cmap.info.height) for c in range(cmap.info.width) if lethal[r, c] and (r, c) in grid)
    print(f'  t={tm:.1f} 致命格 {total} | 被去畸变点云解释 {explained} ({100*explained/max(1,total):.0f}%)')

    # 致命格"厚度"：到最近自由格(0)的棋盘距离。细线=1~2，厚块=大
    free = g == 0
    from scipy import ndimage
    d = ndimage.distance_transform_edt(free)
    thick = d[lethal]
    print(f'    厚度分布: 均值{thick.mean():.2f}格 中位{np.median(thick):.0f}格 最大{thick.max():.0f}格 | '
          f'<=2格占{(thick<=2).mean()*100:.0f}% | >=5格占{(thick>=5).mean()*100:.0f}%')
