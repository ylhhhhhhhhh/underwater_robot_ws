#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cabin_interface.msg import ControlCmd, ControlMove
from geometry_msgs.msg import Vector3
import sys
from pathlib import Path

script_folder = Path(__file__).parent
sys.path.insert(0, str(script_folder))

import pid

class Yaw_pid(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.cmd_sub = self.create_subscription(
            ControlCmd,
            'command/cmd',
            self.cmd_callback,
            10
        )
        self.move_sub = self.create_subscription(
            ControlMove,
            'command/move',
            self.move_callback,
            10
        )
        self.yaw_sub = self.create_subscription(
            Vector3,
            'tf/euler_angles',
            self.yaw_callback,
            10
        )

        self.yaw_pid = pid.PIDController(
            kp=0.5,
            ki=0.2,
            kd=0.3,
            max_out=33.0,
            min_out=-33.0,
            max_i=10.0,
            min_i=-10.0,
            error_tolerance=3.0
        )
        self.move_pub = self.create_publisher(ControlMove, 'command/pid/move', 10)
        self.cached_move = ControlMove()

        self.pid_enable = False
        self.yaw = 0.0
        self.target_yaw = 0.0

        self.tim = self.create_timer(0.02, self.timer_callback)

    def pid_param_init(self):
        self.target_yaw = self.yaw
        self.yaw_pid.set_target(self.target_yaw)
        self.yaw_pid.reset()

    def cmd_callback(self, msg):
        if msg.pid_enable is True:
            self.pid_param_init()
            self.pid_enable = True
        else:
            self.pid_enable = False
            self.yaw_pid.reset()

    def yaw_callback(self, msg):
        raw_yaw = msg.z
        self.yaw = raw_yaw % 360.0

    def move_callback(self, msg):
        self.cached_move = msg
        if self.pid_enable:
            delta_yaw = float(msg.moment.z) * 0.06
            self.target_yaw = (self.target_yaw + delta_yaw) % 360.0
            self.yaw_pid.set_target(self.target_yaw)

    def timer_callback(self):
        out_msg = self.cached_move
        if self.pid_enable:
            yaw_torque = self.yaw_pid.compute(self.yaw)
            out_msg.moment.z = -yaw_torque
        self.move_pub.publish(out_msg)
        # self.get_logger().info(f"target:{self.target_yaw},output:{out_msg.moment.z}")



def main(args=None):
    rclpy.init(args=args)
    node = Yaw_pid("yaw_pid_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
