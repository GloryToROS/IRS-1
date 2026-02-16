import math
import serial
import rclpy
from time import sleep, time
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
from robot_driver.arduino_driver import ArduinoDriver


class RobotDriverNode(Node):
    def __init__(self):
        super().__init__("robot_driver")

        # Инициализация параметров
        self.TICKS_PER_REV = 2500.0
        self.WHEEL_DIAMETER = 0.065  # 6.5 см
        self.WHEEL_BASE = 0.16  # 16 см
        self.CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER

        # Железо
        self.arduino = ArduinoDriver("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0")

        # Состояние одометрии
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Хранение абсолютных тиков для вычисления дельты
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()

        # ROS интерфейсы
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = TransformBroadcaster(self)

        # Таймер обновления (20 Гц)
        self.create_timer(0.05, self.update)
        self.get_logger().info("Драйвер робота IRS-1 запущен")

    def cmd_callback(self, msg):
        self.arduino.proceed_command(msg)

    def update(self):
        # Получаем данные (gy25 в скетче пустой, поэтому игнорируем)
        _, enc, _ = self.arduino.get_robot_data()

        if enc is None or len(enc) < 2:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Если это первый запуск, просто инициализируем счетчики
        if self.last_left_ticks is None:
            self.last_left_ticks = enc[0]
            self.last_right_ticks = enc[1]
            return

        # 1. Считаем сколько тиков проехало каждое колесо с прошлого раза
        d_left_ticks = enc[0] - self.last_left_ticks
        d_right_ticks = enc[1] - self.last_right_ticks

        # 2. Переводим тики в пройденные метры
        dist_left = (d_left_ticks / self.TICKS_PER_REV) * self.CIRCUMFERENCE
        dist_right = (d_right_ticks / self.TICKS_PER_REV) * self.CIRCUMFERENCE

        # 3. Вычисляем общее перемещение робота (ds) и изменение угла (dyaw)
        ds = (dist_left + dist_right) / 2.0
        dyaw = (dist_right - dist_left) / self.WHEEL_BASE

        # 4. Обновляем глобальные координаты x, y, yaw (интегрирование)
        # Используем yaw + dyaw/2 для более точной аппроксимации дуги
        self.x += ds * math.cos(self.yaw + dyaw / 2.0)
        self.y += ds * math.sin(self.yaw + dyaw / 2.0)
        self.yaw += dyaw

        # Сохраняем значения для следующего цикла
        self.last_left_ticks = enc[0]
        self.last_right_ticks = enc[1]

        # 5. Публикуем данные
        self.publish_data(now)

    def publish_data(self, now):
        # Превращаем угол yaw в кватернион для ROS
        q = euler2quat(0, 0, self.yaw)

        # Сообщение одометрии
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]

        self.odom_pub.publish(odom)

        # Сообщение TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_pub.sendTransform(t)


def main():
    rclpy.init()
    node = RobotDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()