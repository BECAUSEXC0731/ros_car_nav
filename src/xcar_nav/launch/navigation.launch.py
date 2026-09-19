import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Nav2 导航启动（AMCL 定位 + 已保存地图）。

    组成：
      1) 机器人本体  xcar_bringup/robot_base.launch.py
         —— URDF 静态 TF + EKF 里程计（发布 odom -> base_footprint）
      2) Nav2 官方 bringup（slam:=False）
         —— map_server 加载地图 + AMCL 定位（发布 map -> odom）
            + planner / controller / behaviors / bt_navigator / velocity_smoother
      3) 可选 RViz

    TF 权责（不要同时跑 slam_pkg 的建图，否则会抢发 map -> odom）：
      map --(AMCL)--> odom --(EKF)--> base_footprint --(URDF)--> base_link

    前置条件：下位机已经在发布 /scan、/odom、/imu 三个话题。
    """
    nav_dir = get_package_share_directory('xcar_nav')
    bringup_dir = get_package_share_directory('xcar_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    nav2_delay = LaunchConfiguration('nav2_delay')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_dir, 'maps', 'qinshi.yaml'),
        description='地图 yaml 的绝对路径')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 参数文件')
    declare_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz2（用 2D Pose Estimate 给初始位姿 / 2D Goal Pose 下目标点）')
    declare_composition = DeclareLaunchArgument(
        'use_composition', default_value='false',
        description='true=所有 Nav2 节点合成一个进程(省内存)；false=每节点独立进程(便于排查)')
    declare_delay = DeclareLaunchArgument(
        'nav2_delay', default_value='3.0',
        description='等待机器人本体(URDF/EKF)起来的秒数，再启动 Nav2')

    # 1) 机器人本体：URDF + EKF
    robot_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot_base.launch.py')))

    # 2) Nav2：定位 + 导航（官方 bringup，slam:=False 表示用 AMCL + 静态地图）
    #
    # 坑：Nav2 内部用 PythonExpression(['not ', use_composition]) 判断是否组合启动，
    #     如果传小写 'false'，表达式会变成 `not false` → NameError: name 'false' is not defined。
    #     所以这里统一把用户输入的 true/false 规范成首字母大写的 True/False 再传进去。
    composition_normalized = PythonExpression(
        ["'", use_composition, "'.capitalize()"])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': composition_normalized,
        }.items())

    # 延迟启动：等 TF(/scan、odom->base_footprint) 就绪，避免 AMCL 激活失败
    nav2_delayed = TimerAction(period=nav2_delay, actions=[nav2])

    # 3) 可选 RViz（用 Nav2 官方视图：地图 / 代价地图 / 路径 / 粒子云）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav_dir, 'rviz', 'nav2_view.rviz')],
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_rviz,
        declare_composition,
        declare_delay,
        robot_base,
        nav2_delayed,
        rviz,
    ])
