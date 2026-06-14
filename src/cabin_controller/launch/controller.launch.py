from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cabin_controller",
            namespace = "red",
            executable= "yaw_pid.py",
            name= "yaw_pid_node",
            output= "screen",
        ),
        Node(
            package="cabin_controller",
            namespace = "red",
            executable= "thruster_controller",
            name= "thruster_controller",
            output= "screen",  
        ),
        Node(
            package="cabin_controller",
            namespace = "red",
            executable= "pwm_controller",
            name= "pwm_controller",
            output= "screen",
        ),
        Node(
            package="cabin_controller",
            namespace = "red",
            executable= "serial_to_mcu.py",
            name= "serial_to_mcu",
            output= "screen",
        )
    ])
