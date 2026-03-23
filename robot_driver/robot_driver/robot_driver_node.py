import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import serial
import tf2_ros
import math
from transforms3d.euler import euler2quat


class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver')

        # --- НАСТРОЙКИ ---
        self.port = "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
        self.baud = 9600

        # --- ПОДКЛЮЧЕНИЕ ---
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            self.get_logger().info(f"CONNECTED: {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"SERIAL ERROR: {e}")
            return

        # --- ФИЗИКА РОБОТА ---
        self.TICKS_PER_REV = 2500.0
        self.WHEEL_BASE = 0.16
        self.CIRCUMFERENCE = math.pi * 0.065

        # --- СОСТОЯНИЕ ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_left_ticks = None
        self.last_right_ticks = None

        # --- ROS 2 ИНТЕРФЕЙСЫ ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # --- ТАЙМЕРЫ ---
        # # запрос данных: 10 Гц
        # self.create_timer(0.1, self.request_data)

        # чтение буфера
        self.create_timer(0.01, self.read_serial)

        # публикация в ROS и консоль
        self.create_timer(0.05, self.publish_diagnostics)

    def cmd_callback(self, msg):
        """Обработка команд движения (приоритет!)"""
        # Лог в сыром виде
        self.get_logger().info(f"CMD: lin={msg.linear.x:.2f} ang={msg.angular.z:.2f}")

        v = msg.linear.x * 100.0  # м/с -> см/с
        w = msg.angular.z

        v_left = int(v - w * (self.WHEEL_BASE * 100.0) / 2.0)
        v_right = int(v + w * (self.WHEEL_BASE * 100.0) / 2.0)

        try:
            cmd = f"N {v_left} {v_right}\n"
            self.ser.write(cmd.encode())
            self.ser.flush() # раскомментировать, если будут задержки при отклике на команды
            # self.get_logger().info(f"Command sent")
        except Exception as e:
            self.get_logger().error(f"WRITE FAIL: {e}")

    def request_data(self):
        """Запрос данных одометрии"""
        try:
            # Если канал свободен - шлем запрос
            if self.ser.out_waiting == 0:
                self.ser.write(b"g\n")
        except:
            pass

    def read_serial(self):
        """Чтение ответов Arduino"""
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line.startswith("ENC:"):
                    parts = line.split()
                    if len(parts) >= 3:
                        l_ticks = int(parts[1])
                        r_ticks = int(parts[2])
                        self.calculate_step(l_ticks, r_ticks)
            except:
                pass

    def calculate_step(self, l_ticks, r_ticks):
        """Математика одометрии"""
        if self.last_left_ticks is None:
            self.last_left_ticks = l_ticks
            self.last_right_ticks = r_ticks
            return

        d_l = ((l_ticks - self.last_left_ticks) / self.TICKS_PER_REV) * self.CIRCUMFERENCE
        d_r = ((r_ticks - self.last_right_ticks) / self.TICKS_PER_REV) * self.CIRCUMFERENCE

        ds = (d_l + d_r) / 2.0
        dyaw = (d_r - d_l) / self.WHEEL_BASE

        self.x += ds * math.cos(self.yaw + dyaw / 2.0)
        self.y += ds * math.sin(self.yaw + dyaw / 2.0)
        self.yaw += dyaw

        self.last_left_ticks = l_ticks
        self.last_right_ticks = r_ticks

    def publish_diagnostics(self):
        """Публикация в ROS и консоль"""
        # создание времени
        ros_now = self.get_clock().now()
        future = (ros_now + rclpy.duration.Duration(seconds=0.2)).to_msg()
        now = ros_now.to_msg()

        q = euler2quat(0, 0, self.yaw)

        # одометрия
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.w = q[0]
        o.pose.pose.orientation.x = q[1]
        o.pose.pose.orientation.y = q[2]
        o.pose.pose.orientation.z = q[3]
        self.odom_pub.publish(o)

        # # трансформы
        # # больше не нужен, так как его публикует ekf
        # t = TransformStamped()
        # t.header = o.header
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.rotation =         # # трансформы
        # t = TransformStamped()
        # t.header = o.header
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.rotation = o.pose.pose.orientation
        # # self.tf_broadcaster.sendTransform(t)

        # трансформ для лидара
        laser_t = TransformStamped()
        laser_t.header.stamp = now
        laser_t.header.frame_id = 'base_link'
        laser_t.child_frame_id = 'laser_frame'
        laser_t.transform.translation.x = -0.03  # смещение лидара
        laser_t.transform.translation.y = 0.0
        laser_t.transform.translation.z = 0.15
        laser_t.transform.rotation.w = 1.0  # поворот, если лидар развернут
        # self.tf_broadcaster.sendTransform(laser_t)

        # self.tf_broadcaster.sendTransform([t, laser_t])  # Шлем списком!
        self.tf_broadcaster.sendTransform(laser_t)
        # # self.tf_broadcaster.sendTransform(t)

        # логирование координат
        # self.get_logger().info(f"X={self.x:.3f} Y={self.y:.3f} Yaw={self.yaw:.2f}")


def main():
    rclpy.init()
    node = RobotDriverNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()