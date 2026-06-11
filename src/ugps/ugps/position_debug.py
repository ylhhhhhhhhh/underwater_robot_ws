#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformBroadcaster, TransformStamped
import sys
import termios
import tty
import time

# 键盘控制
KEY_UP = 'w'
KEY_DOWN = 's'
KEY_LEFT = 'a'
KEY_RIGHT = 'd'
KEY_U = 'u'
KEY_J = 'j'
KEY_RESET = 'r'
KEY_QUIT = 'q'

class PositionDebugNode(Node):
    def __init__(self):
        super().__init__('position_debug_node')

        # 发布官方话题：/ugps_position
        self.pub = self.create_publisher(PointStamped, "/ugps_position", 10)

        # 直接发布 TF！一步到位！
        self.tf_broadcaster = TransformBroadcaster(self)

        # 键盘控制的位置
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.step = 0.1

        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.get_logger().info("===== 键盘控制 UGPS 位置 =====")
        self.get_logger().info("w/s x±0.1 | a/d y±0.1 | u/j z±0.1 | r重置 | q退出")

    def publish(self):
        # 发布 PointStamped 官方消息（RViz 直接识别）
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ugps_link"
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.z
        self.pub.publish(msg)

        # 直接发布 map → ugps_link TF（机器人能动的关键！）
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "ugps_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.w = 1.0  # 无旋转
        self.tf_broadcaster.sendTransform(t)

    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.read_key()

                if key == KEY_UP:
                    self.x += self.step
                elif key == KEY_DOWN:
                    self.x -= self.step
                elif key == KEY_RIGHT:
                    self.y += self.step
                elif key == KEY_LEFT:
                    self.y -= self.step
                elif key == KEY_U:
                    self.z += self.step
                elif key == KEY_J:
                    self.z -= self.step
                elif key == KEY_RESET:
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                elif key == KEY_QUIT:
                    break

                self.publish()
                self.get_logger().info(f"UGPS: x={self.x:.2f} y={self.y:.2f} z={self.z:.2f}")
                time.sleep(0.05)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = PositionDebugNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
