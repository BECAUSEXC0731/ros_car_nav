#!/usr/bin/env python3
"""单独启动纯激光定位节点（调试用）。

只想验证定位是否正常、不起整套 Nav2 时用：
    ros2 launch xcar_jiguang lidar_loc.launch.py

前提：
  * 已有 map_server 或任意节点在发布 /map（transient_local）
  * 已有 TF: odom -> base_footprint（EKF / odom2tf）
  * 已有 /scan_ts（scan_repub）或把 scan_topic 参数改回 /scan
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('xcar_jiguang')
    nav_dir = get_package_share_directory('xcar_nav')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    with_rviz = LaunchConfiguration('use_rviz')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'lidar_loc.yaml'),
        description='lidar_loc 参数文件')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_dir, 'maps', 'shiyan.yaml'),
        description='调试用静态地图')
    declare_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='是否启动 RViz2')

    # 调试时自己起一个 map_server，省得先跑整套导航
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}])

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_debug',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}])

    lidar_loc = Node(
        package='xcar_jiguang',
        executable='lidar_loc',
        name='lidar_loc',
        output='screen',
        parameters=[params_file])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav_dir, 'rviz', 'nav2_view.rviz')],
        condition=IfCondition(with_rviz))

    return LaunchDescription([
        declare_params,
        declare_map,
        declare_rviz,
        map_server,
        map_lifecycle,
        lidar_loc,
        rviz,
    ])
