import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('robot_description')
    urdf_path = os.path.join(pkg, 'urdf', 'robot.xacro')
    rviz_config_path = os.path.join(pkg,'rviz','auv.rviz')
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(urdf_path),description='加载模型文件路径'
    )
    #通过文件路径，获取内容并转换为参数
    command_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result,value_type = str)

    return LaunchDescription([
        action_declare_arg_mode_path,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_value}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'ugps_link']
        ),
    ])
