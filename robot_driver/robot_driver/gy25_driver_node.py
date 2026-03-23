import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import threading


class GyroNode(Node):
    def __init__(self):
        super().__init__('gyro_node')

        # Параметры подключения
        self.port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
        self.baud = 115200

        try:
            # Ставим небольшой timeout, чтобы read() не зависал вечно при выходе
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to Gyro on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Gyro: {e}")
            return

        self.publisher_ = self.create_publisher(Float32, 'gyro/yaw', 10)
        self.gyro_window = [0] * 8

        # Запускаем бесконечный цикл чтения в отдельном потоке
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        """Метод выполняется в отдельном потоке"""
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    byte = self.ser.read(1)
                    if not byte:
                        continue

                    self.gyro_window.pop(0)
                    self.gyro_window.append(ord(byte))

                    # Проверка пакета: [0xAA, high, low, ..., 0x55]
                    if self.gyro_window[0] == 0xAA and self.gyro_window[7] == 0x55:
                        raw = (self.gyro_window[1] << 8) | self.gyro_window[2]
                        if raw > 32767: raw -= 65536
                        yaw_deg = raw / 100.0

                        msg = Float32()
                        msg.data = yaw_deg
                        self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error in read loop: {e}")
                break


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