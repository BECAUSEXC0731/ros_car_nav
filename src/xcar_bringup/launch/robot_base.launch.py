import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('xcar_bringup')

    use_ekf = LaunchConfiguration('use_ekf')

    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
#是否使用EFK
    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='是否启动EFK')

    # 1. URDF -> 静态 TF
    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'urdf2tf.launch.py')))

    # 2a. IMU 帧重发布,补偿 MPU 安装朝向
    imu_repub = Node(
        package='xcar_bringup',
        executable='imu_repub.py',
        name='imu_repub',
        output='screen',
        condition=IfCondition(use_ekf))

    # 2b. /odom 协方差修正
    odom_cov_repub = Node(
        package='xcar_bringup',
        executable='odom_cov_repub.py',
        name='odom_cov_repub',
        output='screen',
        condition=IfCondition(use_ekf))

    # 2c. /scan 时间戳归一
    scan_repub = Node(
        package='xcar_bringup',
        executable='scan_repub.py',
        name='scan_repub',
        output='screen',
        # reliable=True：用 Reliable 发布 /scan_ts。
        # 不加的话（默认 BEST_EFFORT）默认配置的 RViz、ros2 topic echo 这类
        # Reliable 订阅者收不到数据（表现为“话题能选但一个点都没有”）。
        # 订阅端（代价地图、lidar_loc）用 SensorDataQoS，两种 QoS 都兼容。
        parameters=[{'restamp': True, 'reliable': True}])

    # 2d. 转镜雷达运动畸变补偿：/scan_ts -> /scan_cloud（PointCloud2，激光系）
    # 转镜雷达扫一圈 0.2s，车旋转时一圈内的 224 束点各自对应的车位姿不同，
    # 单一时间戳会整圈“糊”开；此节点按每束点真实采样时刻去畸变，给代价地图用。
    scan_deskew = Node(
        package='xcar_bringup',
        executable='scan_deskew.py',
        name='scan_deskew',
        output='screen')

    # 2c. EKF：融合 /odom_cov + /imu_rotated，发布 odom -> base_footprint
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odom/filtered')],
        condition=IfCondition(use_ekf))

    # 2d. 降级方案：直接把 /odom 转成 TF
    odom2tf = Node(
        package='xcar_bringup',
        executable='odom2tf',
        name='odom2tf',
        output='screen',
        parameters=[{'restamp': True}],
        condition=UnlessCondition(use_ekf))

    return LaunchDescription([
        declare_use_ekf,
        urdf2tf,
        imu_repub,
        odom_cov_repub,
        scan_repub,
        scan_deskew,
        ekf_node,
        odom2tf,
    ])
