from __future__ import print_function
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import requests
import json

class UGPSNODE(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.pub = self.create_publisher(PointStamped, "/ugps_position", 10)
        
        self.base_url = "http://192.168.7.1"
        self.create_timer(0.1, self.publish_ugps)

    def get_data(self, url):
        try:
            r = requests.get(url, timeout=0.5)
        except requests.exceptions.RequestException as exc:
            self.get_logger().warn(f"请求异常: {exc}")
            return None

        if r.status_code != 200:
            self.get_logger().warn(f"请求错误: {r.status_code}")
            return None

        return r.json()

    def publish_ugps(self):
        data = self.get_data(f"{self.base_url}/api/v1/position/acoustic/filtered")
        if not data:
            return

        raw_x = data["x"]
        raw_y = data["y"]
        raw_z = data["z"]

        msg = PointStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.point.x = float(raw_x)
        msg.point.y = float(raw_y)
        msg.point.z = float(raw_z)

        self.pub.publish(msg)
        # self.get_logger().info(f"发布坐标: x={raw_x:.2f} y={raw_y:.2f} z={raw_z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = UGPSNODE("ugps_node")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
