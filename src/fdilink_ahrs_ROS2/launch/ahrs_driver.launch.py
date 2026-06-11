import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ahrs_driver=Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        parameters=[
            {
                'if_debug_': False,
                'serial_port_': '/dev/wheeltec_FDI_IMU_GNSS',
                'serial_baud_': 921600,
                'imu_topic': '/imu',
                'imu_frame_id_': 'gyro_link',
                'Euler_angles_topic': '/euler_angles',
                'device_type_': 1,
                'publish_acc': True,    #是否发布加速度
                'publish_gyro': True,   #是否发布角加速度
                'publish_euler': True   #是否发布欧拉角
            }
        ],
        output="screen"
    )

    launch_description = LaunchDescription()
    launch_description.add_action(ahrs_driver)
    return launch_description
