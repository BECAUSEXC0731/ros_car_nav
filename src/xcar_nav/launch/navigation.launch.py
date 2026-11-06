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

    nav_dir = get_package_share_directory('xcar_nav')
    bringup_dir = get_package_share_directory('xcar_bringup')
    jiguang_dir = get_package_share_directory('xcar_jiguang')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    nav2_delay = LaunchConfiguration('nav2_delay')

#修改启动地图
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_dir, 'maps', 'shiyan.yaml'),
        description='地图路径')
#加载nav2参数
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 参数文件')
#启动rviz2
    declare_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false')

    declare_composition = DeclareLaunchArgument(
        'use_composition', default_value='false',
        description='true=所有 Nav2 节点合成一个进程(省内存)；false=每节点独立进程(便于排查)')
    
    declare_delay = DeclareLaunchArgument(
        'nav2_delay', default_value='3.0',
        description='等待机器人本体起来的秒数，再启动 Nav2')

    # 1) 机器人本体底盘
    robot_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot_base.launch.py')))

    # 2) 静态地图服务器
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml, 'use_sim_time': False}])

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}])

    # 3) 纯激光地图匹配定位
    lidar_loc = Node(
        package='xcar_jiguang',
        executable='lidar_loc',
        name='lidar_loc',
        output='screen',
        parameters=[os.path.join(jiguang_dir, 'config', 'lidar_loc.yaml')])

    # 4) Nav2 导航栈
    composition_normalized = PythonExpression(
        ["'", use_composition, "'.capitalize()"])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': composition_normalized,
        }.items())

    # 延迟启动
    nav2_delayed = TimerAction(period=nav2_delay, actions=[nav2])

    # 3) RViz
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
        map_server,
        map_lifecycle,
        lidar_loc,
        nav2_delayed,
        rviz,
    ])
