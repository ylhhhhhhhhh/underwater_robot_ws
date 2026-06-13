import rclpy
from rclpy.node import Node
from cabin_interface.msg import ControlCmd, ControlMove
from geometry_msgs.msg import Vector3
from cabin_controller.scripts import pid


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
        self.pid_move = ControlMove()

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
            self.get_logger().info("Yaw PID 开启")
        else:
            self.pid_enable = False
            self.yaw_pid.reset()
            self.get_logger().info("Yaw PID 关闭")

    def yaw_callback(self, msg):
        raw_yaw = msg.z
        self.yaw = raw_yaw % 360.0

    def move_callback(self, msg):
        if not self.pid_enable:
            self.move_pub.publish(msg)
            return
        
        delta_yaw = float(msg.moment[2]) * 0.5
        self.target_yaw = (self.yaw + delta_yaw) % 360.0
        self.yaw_pid.set_target(self.target_yaw)

    def timer_callback(self):
        if self.pid_enable:
            yaw_torque = self.yaw_pid.compute(self.yaw)
            self.pid_move.moment[2] = yaw_torque
            self.move_pub.publish(self.pid_move)
        else:
            self.pid_move.moment[2] = 0.0
            self.move_pub.publish(self.pid_move)


def main(args=None):
    rclpy.init(args=args)
    node = Yaw_pid("yaw_pid_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
