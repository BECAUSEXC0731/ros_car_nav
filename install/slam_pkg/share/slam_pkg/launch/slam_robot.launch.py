import os
import launch
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    #1雷达驱动节点
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
            get_package_share_directory('lidar_pkg'), 'launch','lidar.launch.py')
        )
    )

    #2.机器人模型和雷达的TF关系
    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
            get_package_share_directory('xcar_bringup'), 'launch', 'urdf2tf.launch.py')
            )
    )

    #3.里程计TF

    odom2tf = launch_ros.actions.Node(
        package='xcar_bringup',
        executable='odom2tf',
        output='screen'
    )


    slam_params = {
        "use_sim_time": False,
        "base_frame": "base_footprint",
        "odom_frame": "odom",
        "map_frame": "map"
    }
    slam_cmd = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[slam_params]
    )

   

    ld = LaunchDescription()
    ld.add_action(slam_cmd)
     
    ld.add_action(lidar_launch)
    ld.add_action(urdf2tf)
    ld.add_action(odom2tf)

    return ld
