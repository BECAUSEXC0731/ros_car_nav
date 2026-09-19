import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """slam_robot.launch.py 的便捷写法：建图 + RViz。

    保留这个文件名是为了兼容过去的启动习惯；
    真正的逻辑只有一份，在 slam_robot.launch.py 里。
    """
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_pkg'),
                             'launch', 'slam_robot.launch.py')),
            launch_arguments={'use_rviz': 'true'}.items()),
    ])
