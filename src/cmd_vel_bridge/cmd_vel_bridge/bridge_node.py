import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        # تأكد من أن الـ Port صحيحة
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
        
        self.wheel_base = 0.63  # تم التعديل حسب بياناتك (63cm)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.safety_check)

    def cmd_callback(self, msg):
        self.last_msg_time = self.get_clock().now()

        # القيم تعود لأصلها بدون سالب
        v = msg.linear.x  
        w = msg.angular.z 

        left_v  = v - (w * self.wheel_base / 2.0)
        right_v = v + (w * self.wheel_base / 2.0)

        command = f"{left_v:.3f} {right_v:.3f}\n"
        self.ser.write(command.encode())

    def safety_check(self):
        # توقف الروبوت إذا انقطع الـ cmd_vel[cite: 1]
        now = self.get_clock().now()
        dt = (now - self.last_msg_time).nanoseconds / 1e9
        if dt > 0.5:
            self.ser.write(b"0.0 0.0\n")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()