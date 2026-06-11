#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from cabin_interface.msg import Pwm
import time

# 自定义通信协议配置
FRAME_HEADER = b'\xAA\x55'  # 帧头
CMD_TYPE_PWM = 0x01         # PWM指令类型
FRAME_LEN_PWM = 17          # 6路PWM帧总长度：2+1+1+12+1=18
PWM_MIN = 1100              # PWM最小值（us）
PWM_MAX = 1900              # PWM最大值（us）
PWM_CHANNELS = 6            # 固定6路PWM

class SerialPWMNode(Node):
    def __init__(self):
        super().__init__('serial_to_mcu')
        # 串口配置参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyS9'),    # 串口设备名
                ('baudrate', 115200),            # 串口波特率
                ('timeout', 1.0)                 # 串口超时时间
            ]
        )
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        # 2. 初始化串口
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            self.get_logger().info(f"串口初始化成功：{port} ({baud}bps)")
        except Exception as e:
            self.get_logger().error(f"串口初始化失败：{str(e)}")
            raise SystemExit(1)
        
        self.sub_pwm = self.create_subscription(
            Pwm,
            'pwm',  
            self.pwm_callback,
            10  
        )
        self.latest_frame = None
        self.send_timer = self.create_timer(0.1, self.timer_send_callback)
        

    def us_to_hex(self, pwm_us):
        """将1100~1900us的PWM值拆分为高低8位（用于串口传输）"""
        # 安全限制PWM值范围
        pwm_us = max(min(pwm_us, PWM_MAX), PWM_MIN)
        # 拆分为高低8位（1100=0x044C，1900=0x076C）
        high_byte = (pwm_us >> 8) & 0xFF
        low_byte = pwm_us & 0xFF
        return high_byte, low_byte

    def calc_checksum(self, data):
        """计算累加和校验位（低8位）"""
        checksum = sum(data) & 0xFF
        return checksum

    def pwm_callback(self, msg):
        """PwmStamped话题回调：解析6路PWM并发送串口帧"""
        # 1. 解析自定义消息的6个PWM字段
        try:
            left_front = msg.left_front    # 左前
            right_front = msg.right_front    # 右前
            left_back = msg.left_back    # 左后
            right_back = msg.right_back    # 右后
            left_up = msg.left_up      # 上左前
            right_up = msg.right_up      # 上右后

            # 整理为6路PWM列表（顺序与STM32接收解析顺序一致）
            pwm_list = [
                left_front,
                right_front,
                left_back,
                right_back,
                left_up,
                right_up
            ]
        except AttributeError as e:
            self.get_logger().error(f"解析PWM消息字段失败：{str(e)}")
            return

        # 2. 构造串口协议帧
        frame = bytearray(FRAME_LEN_PWM)
        frame[0:2] = FRAME_HEADER       # 帧头 0xAA 0x55
        frame[2] = FRAME_LEN_PWM        # 帧长度 18
        frame[3] = CMD_TYPE_PWM         # 指令类型 0x01

        # 3. 填充6路PWM数据（每路2字节，高低位）
        offset = 4  # 数据段起始偏移
        for pwm_us in pwm_list:
            high, low = self.us_to_hex(pwm_us)
            frame[offset] = high
            frame[offset+1] = low
            offset += 2

        # 4. 计算并填充校验位（最后1字节）
        frame[-1] = self.calc_checksum(frame[:-1])
        self.latest_frame = frame

    def timer_send_callback(self):
        """定时器回调：0.1秒触发，只发送最新缓存的帧"""
        if self.latest_frame is not None:
            try:
                self.ser.write(self.latest_frame)
                self.get_logger().debug(f"发送串口帧(16进制)：{self.latest_frame.hex()}")
            except Exception as e:
                self.get_logger().error(f"串口发送失败：{str(e)}")

    def destroy_node(self):
        """节点销毁时关闭串口（完全不变）"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()

def main(args=None):
    # 初始化ROS2节点（完全不变）
    rclpy.init(args=args)
    node = SerialPWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("接收到退出信号，正在关闭节点...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()