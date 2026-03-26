import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import serial
import math
import time
import tf2_ros


class GyroNode(Node):
    def __init__(self):
        super().__init__('gyro_node')

        self.port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
        self.baud = 115200
        self.frame_id = 'imu_link'

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"CONNECTED to gyro")
        except Exception as e:
            self.get_logger().error(f"SERIAL ERROR: {e}")
            return

        # ROS
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Буфер
        self.window = [0] * 8

        # Калибровка
        self.yaw_offset = 0.0
        self.calibrated = False

        # Таймер вместо потока
        self.create_timer(0.001, self.read_loop)
        self.create_timer(0.05, self.publish_tf)

        # Отдельный флаг, чтобы не спамить
        self.last_log_time = time.time()

    # --- ВСПОМОГАТЕЛЬНОЕ ---

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def yaw_to_quaternion(self, yaw):
        return [
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0)
        ]

    def calibrate(self):
        self.get_logger().info("CALIBRATING (2 sec)...")
        samples = []

        start = time.time()
        while time.time() - start < 2.0:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if len(byte) == 0:
                    continue

                self.window.pop(0)
                self.window.append(ord(byte))

                if self.window[0] == 0xAA and self.window[7] == 0x55:
                    raw = (self.window[1] << 8) | self.window[2]
                    if raw > 32767:
                        raw -= 65536
                    samples.append(raw / 100.0)

        if samples:
            self.yaw_offset = sum(samples) / len(samples)
            self.get_logger().info(f"OFFSET: {self.yaw_offset:.2f}")
        else:
            self.yaw_offset = 0.0
            self.get_logger().warn("Calibration skipped!")

        self.calibrated = True

    # --- ОСНОВНОЙ ЦИКЛ ---

    def read_loop(self):
        self.calibrate()

        last_yaw = None
        last_time = time.time()

        while rclpy.ok():
            try:
                byte = self.ser.read(1)
                if not byte:
                    continue

                # Ждём начало пакета
                if byte[0] != 0xAA:
                    continue

                # Читаем остальные 7 байт
                data = self.ser.read(7)
                if len(data) != 7:
                    continue

                packet = [0xAA] + list(data)

                # Проверка конца пакета
                if packet[7] != 0x55:
                    self.get_logger().warn(f"BAD PACKET: {packet}")
                    continue

                # --- ПАРСИНГ ---
                raw = (packet[1] << 8) | packet[2]
                if raw > 32767:
                    raw -= 65536

                yaw_deg = raw / 100.0 - self.yaw_offset
                yaw_rad = math.radians(yaw_deg)
                yaw_rad = self.normalize_angle(yaw_rad)

                now = time.time()

                # --- УГЛОВАЯ СКОРОСТЬ ---
                if last_yaw is not None:
                    dt = now - last_time
                    if dt > 0:
                        dyaw = yaw_rad - last_yaw

                        # !!! защита от скачков !!!
                        if abs(dyaw) > math.pi:
                            dyaw = 0.0

                        yaw_rate = dyaw / dt
                    else:
                        yaw_rate = 0.0
                else:
                    yaw_rate = 0.0

                last_yaw = yaw_rad
                last_time = now

                # --- ROS ---
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id

                q = self.yaw_to_quaternion(yaw_rad)
                msg.orientation.x = q[0]
                msg.orientation.y = q[1]
                msg.orientation.z = q[2]
                msg.orientation.w = q[3]

                msg.orientation_covariance = [
                    1e6, 0.0, 0.0,
                    0.0, 1e6, 0.0,
                    0.0, 0.0, 0.05
                ]

                msg.angular_velocity.z = yaw_rate
                msg.angular_velocity_covariance = [
                    1e6, 0.0, 0.0,
                    0.0, 1e6, 0.0,
                    0.0, 0.0, 0.05
                ]

                self.pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                break

    # --- TF ---

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05

        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GyroNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()