import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # =========================
        # 🔌 Serial Setup
        # =========================
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # 🔥 Force ESP32 Reset (زي Arduino IDE)
        self.ser.setDTR(False)
        self.ser.setRTS(True)
        time.sleep(0.5)
        self.ser.setDTR(True)

        self.get_logger().info("ESP32 Reset Done")

        # =========================
        # 🔧 Parameters
        # =========================
        self.wheel_base = 0.64   # 🔁 عدل حسب الروبوت (متر)
        self.scale = 100        # 🔁 عدل حسب السرعة

        # =========================
        # 📡 Subscriber
        # =========================
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # =========================
        # ⏱ Safety Timer
        # =========================
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.safety_check)

    # =========================
    # 📥 Callback
    # =========================
    def cmd_callback(self, msg):
        self.last_msg_time = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z

        # Differential drive equations
        left  = v - (w * self.wheel_base / 2.0)
        right = v + (w * self.wheel_base / 2.0)

        # Scaling
        left_cmd  = int(left * self.scale)
        right_cmd = int(right * self.scale)

        # Clamp (اختياري بس مهم)
        left_cmd  = max(min(left_cmd, 200), -200)
        right_cmd = max(min(right_cmd, 200), -200)

        command = f"{left_cmd} {right_cmd}\n"
        self.ser.write(command.encode())

        self.get_logger().info(f"Sent: {command.strip()}")

    # =========================
    # 🛑 Safety Stop
    # =========================
    def safety_check(self):
        now = self.get_clock().now()
        dt = (now - self.last_msg_time).nanoseconds / 1e9

        # لو مفيش cmd_vel لمدة نص ثانية → وقف
        if dt > 0.5:
            self.ser.write(b"0 0\n")


# =========================
# 🚀 Main
# =========================
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()