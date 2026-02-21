import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import math


class GY25Driver(Node):
    def __init__(self):
        super().__init__('gy25_driver')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        # Настройка порта
        port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
        self.ser = serial.Serial(port, 115200, timeout=1)

        # Таймер опроса (100 Гц)
        self.timer = self.create_timer(0.01, self.read_and_publish)
        self.get_logger().info('GY-25 ROS2 Node started!')

    def decode_angle(self, high, low):
        # Формула для GY-25: (High << 8 | Low) / 100.0
        angle = (high << 8 | low) / 100.0
        if angle > 180:
            angle -= 655.35  # Обработка отрицательных углов
        return math.radians(angle)  # ROS использует радианы

    def read_and_publish(self):
        if self.ser.in_waiting >= 8:
            raw = self.ser.read(self.ser.in_waiting)
            for i in range(len(raw) - 8, -1, -1):
                if raw[i] == 0xAA and raw[i + 1] == 0x02 and raw[i + 7] == 0x55:
                    yaw = self.decode_angle(raw[i + 2], raw[i + 3])
                    pitch = self.decode_angle(raw[i + 4], raw[i + 5])
                    roll = self.decode_angle(raw[i + 6], raw[i + 7])

                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'

                    cy = math.cos(yaw * 0.5)
                    sy = math.sin(yaw * 0.5)
                    cp = math.cos(pitch * 0.5)
                    sp = math.sin(pitch * 0.5)
                    cr = math.cos(roll * 0.5)
                    sr = math.sin(roll * 0.5)

                    msg.orientation.w = cy * cp * cr + sy * sp * sr
                    msg.orientation.x = cy * cp * sr - sy * sp * cr
                    msg.orientation.y = sy * cp * sr + cy * sp * cr
                    msg.orientation.z = sy * cp * cr - cy * sp * sr

                    self.publisher_.publish(msg)
                    break

def main(args=None):
    rclpy.init(args=args)
    node = GY25Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()