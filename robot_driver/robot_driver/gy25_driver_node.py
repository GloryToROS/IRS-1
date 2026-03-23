import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math


class GY25Driver(Node):
    def __init__(self):
        super().__init__('gy25_driver')

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        # --- НАСТРОЙКА ПОРТА ---
        port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
        self.ser = serial.Serial(port, 115200, timeout=1)

        # Таймер опроса 100 Гц
        self.timer = self.create_timer(0.01, self.read_and_publish)

        self.get_logger().info('GY-25 ROS2 Node started!')

    # ------------------------------------------
    # Декодирование углов GY-25
    # ------------------------------------------
    def decode_angle(self, high, low):
        angle = (high << 8 | low) / 100.0

        if angle > 180:
            angle -= 655.35

        return math.radians(angle)

    # ------------------------------------------
    # Основной цикл чтения
    # ------------------------------------------
    def read_and_publish(self):
        if self.ser.in_waiting >= 8:
            raw = self.ser.read(self.ser.in_waiting)

            for i in range(len(raw) - 8, -1, -1):
                if raw[i] == 0xAA and raw[i + 1] == 0x02 and raw[i + 7] == 0x55:

                    yaw = self.decode_angle(raw[i + 2], raw[i + 3])
                    pitch = self.decode_angle(raw[i + 4], raw[i + 5])
                    roll = self.decode_angle(raw[i + 6], raw[i + 7])

                    # ------------------------------------------
                    # Создание сообщения IMU
                    # ------------------------------------------
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'

                    # ------------------------------------------
                    # Перевод RPY -> Quaternion
                    # ------------------------------------------
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

                    # ------------------------------------------
                    # 🔥 ВАЖНО: Covariance для robot_localization
                    # ------------------------------------------

                    # Доверяем ориентации
                    msg.orientation_covariance = [
                        0.01, 0.0, 0.0,
                        0.0, 0.01, 0.0,
                        0.0, 0.0, 0.02
                    ]

                    # Угловую скорость НЕ используем
                    msg.angular_velocity_covariance[0] = -1.0

                    # Ускорения НЕ используем
                    msg.linear_acceleration_covariance[0] = -1.0

                    self.publisher_.publish(msg)
                    break


def main(args=None):
    rclpy.init(args=args)
    node = GY25Driver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
