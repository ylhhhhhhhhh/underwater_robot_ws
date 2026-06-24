# Filename: ping360_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gain', default_value='0', description='Gain setting for the sonar.'),
        DeclareLaunchArgument('frequency', default_value='740', description='Frequency of the sonar in kHz.'),
        DeclareLaunchArgument('range_max', default_value='2', description='Maximum range of the sonar in meters.'),
        DeclareLaunchArgument('angle_sector', default_value='360', description='Angle sector for the sonar scan in degrees.'),
        DeclareLaunchArgument('angle_step', default_value='1', description='Step size for the angle in degrees.'),
        DeclareLaunchArgument('image_size', default_value='500', description='Size of the sonar image.'),
        DeclareLaunchArgument('scan_threshold', default_value='100', description='Threshold for the sonar scan.'),
        DeclareLaunchArgument('speed_of_sound', default_value='1500', description='Speed of sound in water in m/s.'),
        DeclareLaunchArgument('image_rate', default_value='100', description='Rate at which images are published.'),
        DeclareLaunchArgument('sonar_timeout', default_value='8000', description='Timeout for the sonar in milliseconds.'),
        DeclareLaunchArgument('publish_image', default_value='True', description='Whether to publish sonar images.'),
        DeclareLaunchArgument('publish_scan', default_value='True', description='Whether to publish sonar scans.'),
        DeclareLaunchArgument('publish_echo', default_value='True', description='Whether to publish sonar echoes.'),
        DeclareLaunchArgument('frame', default_value='sonar', description='Frame ID for the sonar.'),
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0', description='Device port for the sonar.'),
        DeclareLaunchArgument('baudrate', default_value='115200', description='Baud rate for the serial connection.'),
        DeclareLaunchArgument('fallback_emulated', default_value='False', description='Whether to use emulated fallback.'),
        DeclareLaunchArgument('connection_type', default_value='serial', description='Type of connection (serial/udp).'),
        DeclareLaunchArgument('udp_address', default_value='0.0.0.0', description='UDP address for the sonar.'),
        DeclareLaunchArgument('udp_port', default_value='12345', description='UDP port for the sonar.'),

        Node(
            package='ping360_sonar',
            executable='ping360.py',
            name='ping360',
            output='screen',
            parameters=[
                {'gain': LaunchConfiguration('gain')},
                {'frequency': LaunchConfiguration('frequency')},
                {'range_max': LaunchConfiguration('range_max')},
                {'angle_sector': LaunchConfiguration('angle_sector')},
                {'angle_step': LaunchConfiguration('angle_step')},
                {'image_size': LaunchConfiguration('image_size')},
                {'scan_threshold': LaunchConfiguration('scan_threshold')},
                {'speed_of_sound': LaunchConfiguration('speed_of_sound')},
                {'image_rate': LaunchConfiguration('image_rate')},
                {'sonar_timeout': LaunchConfiguration('sonar_timeout')},
                {'publish_image': LaunchConfiguration('publish_image')},
                {'publish_scan': LaunchConfiguration('publish_scan')},
                {'publish_echo': LaunchConfiguration('publish_echo')},
                {'frame': LaunchConfiguration('frame')},
                {'device': LaunchConfiguration('device')},
                {'baudrate': LaunchConfiguration('baudrate')},
                {'fallback_emulated': LaunchConfiguration('fallback_emulated')},
                {'connection_type': LaunchConfiguration('connection_type')},
                {'udp_address': LaunchConfiguration('udp_address')},
                {'udp_port': LaunchConfiguration('udp_port')}
            ]
        )
    ])
