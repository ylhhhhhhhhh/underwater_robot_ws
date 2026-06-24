#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SonarColorConverter(Node):
    def __init__(self):
        super().__init__('sonar_color_converter')
        self.bridge = CvBridge()
        
        # 修复：手动构建传感器数据的QoS配置（替代SensorDataQoS）
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # 传感器数据推荐BEST_EFFORT
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10  # 缓存最新10条数据
        )
        
        # 订阅灰度图（使用手动构建的QoS）
        self.sub = self.create_subscription(
            Image,
            '/scan_image',
            self.image_callback,
            qos_profile  # 替换原来的SensorDataQoS()
        )
        
        # 发布彩色图（QoS保持一致）
        self.pub = self.create_publisher(Image, '/scan_image_color', qos_profile)
        self.get_logger().info("已启动灰度→彩色转换器，监听 /scan_image，发布到 /scan_image_color")

    def image_callback(self, msg):
        try:
            # 转成OpenCV灰度图
            cv_gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            # 应用伪彩映射（和你截图风格一致的COLORMAP_JET）
            cv_color = cv2.applyColorMap(cv_gray, cv2.COLORMAP_JET)
            # 转成ROS彩色图像（bgr8是OpenCV默认彩色格式）
            color_msg = self.bridge.cv2_to_imgmsg(cv_color, 'bgr8')
            color_msg.header = msg.header
            # 发布彩色图
            self.pub.publish(color_msg)
        except Exception as e:
            self.get_logger().error(f"转换失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SonarColorConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # 清理资源
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
