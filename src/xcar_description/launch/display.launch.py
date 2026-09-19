import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """只查看机器人模型 + RViz（不需要底盘、雷达、导航）。

    这里自己启动 robot_state_publisher（而不是去 include xcar_bringup），
    这样 xcar_description 不依赖 xcar_bringup，避免两个包互相依赖。
    """
    description_dir = get_package_share_directory('xcar_description')
    default_model = os.path.join(description_dir, 'urdf', 'xcar.urdf')

    declare_model = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='URDF 的绝对路径')

    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(description_dir, 'rviz', 'display.rviz')],
        output='screen')

    return LaunchDescription([
        declare_model,
        robot_state_publisher,
        rviz2,
    ])
