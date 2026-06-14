import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ahrs_driver=Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        namespace="red",
        parameters=[
            {
                'if_debug_': False,
                'serial_port_': '/dev/wheeltec_FDI_IMU_GNSS',
                'serial_baud_': 921600,
                'imu_topic': 'imu',
                'imu_frame_id_': 'gyro_link',
                'Euler_angles_topic': 'euler_angles',
                'device_type_': 1,
                'publish_acc': True,
                'publish_gyro': True,
                'publish_euler': True
            }
        ],
        output="screen"
    )

    imu_tf_node = Node(
        package="fdilink_ahrs",
        executable="imu_tf_node",
        output="screen",
        namespace="red",
        parameters=[
            {
                "pub_imu_topic": "tf/imu",
                "pub_euler_topic": "tf/euler_angles",
                "imu_output_frame": "imu_base"
            }
        ]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(ahrs_driver)
    launch_description.add_action(imu_tf_node)
    return launch_description
