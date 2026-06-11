from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ping360_sonar",
            executable="ping360_sonar",
            name="ping360_driver",
            output="screen",
            parameters=[
                # 硬件连接
                {"connection_type": "serial"},
                {"serial_port": "/dev/ttyUSB0"},
                {"baudrate": 115200},
                {"fallback_emulated": False},

                # 扫描参数
                {"aperture_deg": 360.0},
                {"step_deg": 2.0},
                {"max_range": 5.0},
                {"gain": 3},

                # 消息开关
                {"publish_raw_sonar": True},
                {"raw_topic_name": "/sonar/raw_ping"},
                {"frame_id": "sonar_link"},
            ]
        )
    ])
