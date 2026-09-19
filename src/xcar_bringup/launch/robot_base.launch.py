import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """机器人本体启动：URDF 静态 TF + 里程计/IMU 融合。

    统一在这里发布 odom -> base_footprint，避免"两个节点抢发同一段 TF"的经典坑：
      use_ekf:=true （默认）EKF 融合 /odom + /imu，抗打滑
      use_ekf:=false        只用 /odom 直通（下位机没有 IMU 时的降级方案）

    建图（slam_pkg）和导航（xcar_nav）都 include 本文件，不再各自重复写一遍。
    """
    bringup_dir = get_package_share_directory('xcar_bringup')
    use_ekf = LaunchConfiguration('use_ekf')

    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='true=EKF 融合 /odom+/imu；false=仅用 /odom 直通(odom2tf)')

    # 1. URDF -> 静态 TF（base_footprint -> base_link -> 雷达/IMU/轮子）
    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'urdf2tf.launch.py')))

    # 2a. IMU 帧重发布（frame_id -> imu_link，补偿 MPU 安装朝向）
    imu_repub = Node(
        package='xcar_bringup',
        executable='imu_repub.py',
        name='imu_repub',
        output='screen',
        condition=IfCondition(use_ekf))

    # 2b. EKF：融合 /odom + /imu_rotated，发布 odom -> base_footprint
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odom/filtered')],
        condition=IfCondition(use_ekf))

    # 2c. 降级方案：直接把 /odom 转成 TF（不要与 EKF 同时运行）
    odom2tf = Node(
        package='xcar_bringup',
        executable='odom2tf',
        name='odom2tf',
        output='screen',
        condition=UnlessCondition(use_ekf))

    return LaunchDescription([
        declare_use_ekf,
        urdf2tf,
        imu_repub,
        ekf_node,
        odom2tf,
    ])
