import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 加载并发布机器人 URDF 模型和 TF（复用 xcar_bringup 的 urdf2tf）
    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xcar_bringup'),
                'launch', 'urdf2tf.launch.py')
        )
    )

    # 2. RViz2 可视化
    rviz_config = os.path.join(
        get_package_share_directory('xcar_description'),
        'rviz', 'display.rviz'
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        urdf2tf,
        rviz2,
    ])
