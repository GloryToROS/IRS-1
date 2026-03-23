import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import struct


class GyroNode(Node):
    def __init__(self):
        super().__init__('gyro_node')

        # Параметры подключения
        self.port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
        self.baud = 115200

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f"Connected to Gyro on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Gyro: {e}")
            return

        self.publisher_ = self.create_publisher(Float32, 'gyro/yaw', 10)

        # Буфер для обработки пакета (8 байт как в вашем коде)
        self.gyro_window = [0] * 8

        # Таймер опроса (100 Гц)
        self.timer = self.create_timer(0.01, self.read_gyro)

    def read_gyro(self):
        if self.ser.in_waiting > 0:
            chunk = self.ser.read(self.ser.in_waiting)
            for byte in chunk:
                self.gyro_window.pop(0)
                self.gyro_window.append(byte)

                # Проверка заголовка 0xAA и конца 0x55
                if self.gyro_window[0] == 0xAA and self.gyro_window[7] == 0x55:
                    # Сборка 16-битного значения (Big Endian)
                    raw = (self.gyro_window[1] << 8) | self.gyro_window[2]

                    # Обработка знака (int16)
                    if raw > 32767:
                        raw -= 65536

                    yaw_deg = raw / 100.0

                    # Публикация
                    msg = Float32()
                    msg.data = yaw_deg
                    self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GyroNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()