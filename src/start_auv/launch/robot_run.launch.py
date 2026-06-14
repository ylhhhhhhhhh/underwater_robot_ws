import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('fdilink_ahrs'), 'launch'), 'ahrs_driver.launch.py'])
    )
    thruster_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('cabin_controller'), 'launch'), 'controller.launch.py'])
    )
    yolo = Node(
        package='yolov5_ros2_rknn',
        executable='yolov5_rknn_node',
        output = 'screen'
    )
    return LaunchDescription([
        imu_launch,
        thruster_controller_launch,
        yolo,
    ])