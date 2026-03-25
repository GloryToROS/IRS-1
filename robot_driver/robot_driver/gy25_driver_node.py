import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import threading
import math
import time


class GyroNode(Node):
    def __init__(self):
        super().__init__('gyro_node')

        self.port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
        self.baud = 115200
        # Frame ID важен для TF (трансформаций) в SLAM/Nav2
        self.frame_id = 'imu_link'

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to Gyro on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

        # Стандартный топик для IMU
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.gyro_window = [0] * 8

        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def euler_to_quaternion(self, yaw):
        """Перевод угла Yaw (в радианах) в кватернион ROS"""
        # Формула для вращения только вокруг оси Z
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def read_loop(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    byte = self.ser.read(1)
                    if not byte: continue

                    self.gyro_window.pop(0)
                    self.gyro_window.append(ord(byte))

                    if self.gyro_window[0] == 0xAA and self.gyro_window[7] == 0x55:
                        raw = (self.gyro_window[1] << 8) | self.gyro_window[2]
                        if raw > 32767: raw -= 65536

                        # Переводим в радианы (стандарт ROS)
                        yaw_rad = math.radians(raw / 100.0)

                        msg = Imu()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = self.frame_id

                        # Заполняем ориентацию
                        msg.orientation = self.euler_to_quaternion(yaw_rad)

                        # Ковариация (важно для EKF!).
                        # Если гироскоп точный, ставим маленькое число по диагонали (Z-ось).
                        # Остальные оси (X, Y) блокируем огромным числом, так как данных нет.
                        msg.orientation_covariance = [
                            1e-9, 0.0, 0.0,
                            0.0, 1e-9, 0.0,
                            0.0, 0.0, 0.01  # Погрешность по Yaw
                        ]

                        # Остальные поля (ускорение, угл. скорость) оставляем 0,
                        # если датчик их не шлет. EKF умеет работать с частичными данными.
                        self.publisher_.publish(msg)
                else:
                    time.sleep(0.005)  # Спим 5мс, чтобы не "съесть" CPU
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                break


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of your node
    node = GyroNode()

    try:
        # Keep the node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()