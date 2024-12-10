import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Int32


class Principal(Node):
    def __init__(self):
        super().__init__('principal')
        self.subscription = self.create_subscription(
            Int32,
            'action',
            self.listener_callback,
            10)
        self.subscription
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.flag = True

    def listener_callback(self, msg):
        action = msg.data
        if self.flag and action != 5:
            self.enviar_datos(str(action) + '\n')
            self.flag = False
            self.timer = self.create_timer(7, self.timer_callback)

    def enviar_datos(self, action):
        self.ser.write(action.encode())

    def timer_callback(self):
        self.flag = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    principal = Principal()
    try:
        rclpy.spin(principal)
    except KeyboardInterrupt:
        pass
    finally:
        principal.destroy_node()
        rclpy.shutdown()
