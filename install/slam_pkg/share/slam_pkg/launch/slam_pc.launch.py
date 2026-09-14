import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_file = os.path.join(get_package_share_directory('wpr_simulation2'), 'rviz', 'slam.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]
    )

    ld = LaunchDescription()
    
    ld.add_action(rviz_cmd)
   

    return ld