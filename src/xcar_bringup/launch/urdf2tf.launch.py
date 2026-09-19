import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """发布 URDF 描述的静态 TF（base_footprint -> base_link -> 雷达/IMU/轮子）。

    说明：本车 URDF 里全部是 fixed joint，没有可动关节，
    因此不需要 joint_state_publisher（已移除）。
    """
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('xcar_description')
    xcar_model_path = urdf_tutorial_path + '/urdf/xcar.urdf'
    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(xcar_model_path),
        description='URDF 的绝对路径')
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False,   # 使用系统时间，而非仿真时间
                     'publish_frequency': 10.0
                     }]
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
    ])
