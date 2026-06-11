#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            namespace = "red",
            executable="joy_node",
            name="joy_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"autorepeat_rate": 0.0}
            ]
        ),
        
        Node(
            package="cabin_teleop",
            namespace = "red",
            executable="joy_controller",
            name="joy_controller_node",
            output="screen",
            emulate_tty=True,
            parameters=[]
        )
    ])
