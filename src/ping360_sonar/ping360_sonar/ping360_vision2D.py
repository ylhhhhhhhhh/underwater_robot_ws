#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cabin_interface.msg import Sonar
import cv2
import numpy as np
import threading

class Ping360OpenCVVis(Node):
    def __init__(self):
        super().__init__("ping360_opencv_vis")
        # 参数
        self.declare_parameter("sonar_topic", "/sonar/raw_ping")
        self.declare_parameter("max_display_range", 10.0)
        self.declare_parameter("img_size", 800)
        self.declare_parameter("intensity_vmin", 0)
        self.declare_parameter("intensity_vmax", 255)

        self.topic_name = self.get_parameter("sonar_topic").get_parameter_value().string_value
        self.max_r = self.get_parameter("max_display_range").get_parameter_value().double_value
        self.img_sz = self.get_parameter("img_size").get_parameter_value().integer_value
        self.vmin = self.get_parameter("intensity_vmin").get_parameter_value().integer_value
        self.vmax = self.get_parameter("intensity_vmax").get_parameter_value().integer_value

        # 存储角度数据 {角度deg: ndarray([dist, inten])}
        self.angle_data = dict()
        self.lock = threading.Lock()
        self.update_flag = False

        # 画布基础参数
        self.center = (self.img_sz // 2, self.img_sz // 2)
        self.max_pixel_r = self.img_sz // 2 - 20

        # 订阅
        self.sub = self.create_subscription(Sonar, self.topic_name, self.callback, 10)
        self.get_logger().info(f"OpenCV声呐可视化启动，订阅{self.topic_name}")

        # 绘图循环定时器
        self.draw_timer = self.create_timer(0.001, self.draw_loop)

    def callback(self, msg):
        # 弧度转0~360度
        deg = np.degrees(msg.angle) % 360.0
        num_samp = msg.num_samples
        r_max = msg.range_max
        inten = np.array(msg.intensities, dtype=np.float32)

        dist_step = r_max / num_samp
        dist_arr = np.arange(num_samp) * dist_step
        mask = dist_arr <= self.max_r
        dist_valid = dist_arr[mask]
        inten_valid = inten[mask]

        data = np.column_stack([dist_valid, inten_valid])
        with self.lock:
            self.angle_data[round(deg, 2)] = data
            self.update_flag = True

    def draw_loop(self):
        if not self.update_flag:
            return
        self.update_flag = False

        # 新建黑色画布
        canvas = np.zeros((self.img_sz, self.img_sz, 3), dtype=np.uint8)
        # 绘制外圈量程圆环
        cv2.circle(canvas, self.center, self.max_pixel_r, (60,60,60), 1)
        # 绘制网格刻度
        for r_scale in [0.25, 0.5, 0.75]:
            r_px = int(self.max_pixel_r * r_scale)
            cv2.circle(canvas, self.center, r_px, (40,40,40), 1)

        with self.lock:
            for ang_deg, point_arr in self.angle_data.items():
                # 角度转换：0度朝右，逆时针增加，匹配Ping360
                rad = np.radians(-ang_deg + 90)
                dists = point_arr[:,0]
                intens = point_arr[:,1]

                # 距离映射像素半径
                r_px_arr = (dists / self.max_r) * self.max_pixel_r
                # 强度归一化转色彩
                norm_int = np.clip((intens - self.vmin)/(self.vmax - self.vmin), 0, 1)

                for r_p, val in zip(r_px_arr, norm_int):
                    x = int(self.center[0] + r_p * np.cos(rad))
                    y = int(self.center[1] - r_p * np.sin(rad))
                    # HSV色彩映射强度
                    hue = int(240 * (1 - val))
                    color = cv2.cvtColor(np.array([[[hue, 255, 255]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0,0]
                    cv2.circle(canvas, (x,y), 1, tuple(map(int, color)), -1)

        # 显示窗口
        cv2.imshow("Ping360 Sonar Scan (OpenCV)", canvas)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Ping360OpenCVVis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("关闭可视化")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
