import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    xcar_bringup_dir = get_package_share_directory(
        'xcar_bringup')
    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [xcar_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    )

    odom2tf = launch_ros.actions.Node(
        package='xcar_bringup',
        executable='odom2tf',
        output='screen'
    )

    lidar_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_pkg'), 'launch', 'lidar.launch.py')
        )
    )


    return launch.LaunchDescription([
        urdf2tf,
        odom2tf,
        lidar_launch,
    ])