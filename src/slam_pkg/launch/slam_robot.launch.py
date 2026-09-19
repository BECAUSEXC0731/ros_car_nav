import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """SLAM 建图主启动。

    组成：机器人本体（URDF + EKF，见 xcar_bringup/robot_base.launch.py）
          + slam_toolbox（订阅下位机的 /scan，发布 map 与 map->odom TF）

    参数：use_rviz:=true 同时打开 RViz
    注意：本文件不启动里程计直通(odom2tf)，也不启动 AMCL/导航，
          避免与 EKF 抢发 odom->base_footprint、与 slam_toolbox 抢发 map->odom。
    """
    slam_dir = get_package_share_directory('slam_pkg')
    bringup_dir = get_package_share_directory('xcar_bringup')

    use_rviz = LaunchConfiguration('use_rviz')
    slam_params = os.path.join(slam_dir, 'config', 'slam_params.yaml')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='是否同时启动 RViz2')

    # 1. 机器人本体：URDF 静态 TF + EKF 里程计融合
    robot_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot_base.launch.py')))

    # 2. slam_toolbox：/scan -> /map，并发布 map -> odom
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params])

    # 3. 可选 RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('xcar_description'), 'rviz', 'display.rviz')],
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        declare_use_rviz,
        robot_base,
        slam,
        rviz,
    ])
