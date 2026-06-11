#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from cabin_interface.msg import Sonar
import threading
from ping360_sonar.sonar_interface import SonarInterface

class Ping360DriverNode(Node):
    def __init__(self):
        super().__init__("ping360_sonar_driver")
        self.declare_parameter("connection_type", "serial", ParameterDescriptor(description="serial / udp"))
        self.declare_parameter("serial_port", "/dev/ttyUSB0", ParameterDescriptor(description="串口设备"))
        self.declare_parameter("baudrate", 115200, ParameterDescriptor(description="串口波特率"))
        self.declare_parameter("udp_ip", "192.168.2.100", ParameterDescriptor(description="UDP设备IP"))
        self.declare_parameter("udp_port", 9090, ParameterDescriptor(description="UDP端口"))
        self.declare_parameter("fallback_emulated", True, ParameterDescriptor(description="连接失败启用仿真"))
        self.declare_parameter("aperture_deg", 360.0, ParameterDescriptor(description="总扫描扇区角度"))
        self.declare_parameter("step_deg", 1.0, ParameterDescriptor(description="单步旋转角度"))
        self.declare_parameter("ensure_divisor", True, ParameterDescriptor(description="步长整除总扇区，适配LaserScan"))
        self.declare_parameter("gain", 3, ParameterDescriptor(description="硬件增益 0~6"))
        self.declare_parameter("frequency", 750000, ParameterDescriptor(description="发射频率Hz"))
        self.declare_parameter("speed_of_sound", 1500.0, ParameterDescriptor(description="水中声速m/s"))
        self.declare_parameter("max_range", 10.0, ParameterDescriptor(description="最大测距量程m"))
        self.declare_parameter("frame_id", "sonar_link", ParameterDescriptor(description="声呐TF坐标系"))
        self.declare_parameter("publish_raw_sonar", True, ParameterDescriptor(description="是否发布自定义原始声呐消息"))
        self.declare_parameter("raw_topic_name", "/sonar/raw_ping", ParameterDescriptor(description="原始声呐话题名"))

        conn_type = self.get_parameter("connection_type").get_parameter_value().string_value
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value
        udp_ip = self.get_parameter("udp_ip").get_parameter_value().string_value
        udp_p = self.get_parameter("udp_port").get_parameter_value().integer_value
        emu = self.get_parameter("fallback_emulated").get_parameter_value().bool_value
        aper_deg = self.get_parameter("aperture_deg").get_parameter_value().double_value
        step_deg = self.get_parameter("step_deg").get_parameter_value().double_value
        ensure_div = self.get_parameter("ensure_divisor").get_parameter_value().bool_value
        gain = self.get_parameter("gain").get_parameter_value().integer_value
        freq = self.get_parameter("frequency").get_parameter_value().integer_value
        sound_speed = self.get_parameter("speed_of_sound").get_parameter_value().double_value
        max_r = self.get_parameter("max_range").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.pub_raw_enable = self.get_parameter("publish_raw_sonar").get_parameter_value().bool_value
        self.raw_topic = self.get_parameter("raw_topic_name").get_parameter_value().string_value

        try:
            self.sonar = SonarInterface(
                port=serial_port,
                baudrate=baud,
                fallback_emulated=emu,
                connection_type=conn_type,
                udp_address=udp_ip,
                udp_port=udp_p
            )
            self.get_logger().info("SonarInterface 硬件实例化成功")
        except Exception as e:
            self.get_logger().fatal(f"声呐初始化失败: {str(e)}")
            raise RuntimeError("Sonar init failed")

        self.sonar.configureAngles(aper_deg, step_deg, ensure_div)
        self.sonar.configureTransducer(gain, freq, sound_speed, max_r)
        self.get_logger().info(
            f"扫描配置完成 | 扇区:{aper_deg}° 步长:{step_deg}° 最大量程:{max_r}m"
        )

        self.pub_raw = None
        if self.pub_raw_enable:
            self.pub_raw = self.create_publisher(Sonar, self.raw_topic, 10)
            self.get_logger().info(f"原始声呐消息开启，话题: {self.raw_topic}")
        else:
            self.get_logger().info("原始声呐消息发布已关闭")

        self.sonar_thread = threading.Thread(target=self._sonar_block_loop, daemon=True)
        self.sonar_thread.start()
        self.get_logger().info("声呐采集线程已启动，串行读取无并发冲突")

    def _sonar_block_loop(self):
        while rclpy.ok():
            try:
                data_ok, scan_finished = self.sonar.read()
            except Exception as e:
                self.get_logger().error(f"声呐读取异常: {str(e)}")
                continue
            if not data_ok:
                continue

            if self.pub_raw_enable and self.pub_raw is not None:
                raw_msg = Sonar()
                raw_msg.header.stamp = self.get_clock().now().to_msg()
                raw_msg.header.frame_id = self.frame_id
                raw_msg.angle = self.sonar.currentAngle()
                raw_msg.sample_period = self.sonar.sample_period
                raw_msg.num_samples = self.sonar.samples
                raw_msg.range_min = 0.0
                raw_msg.range_max = self.sonar.max_range
                raw_msg.intensities = list(self.sonar.data)
                self.pub_raw.publish(raw_msg)
            if scan_finished:
                self.get_logger().info("本轮扇区/360°扫描完成")

def main(args=None):
    rclpy.init(args=args)
    node = Ping360DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到退出信号，关闭驱动")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
