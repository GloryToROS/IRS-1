from time import time, sleep
import serial

import rclpy
from rclpy.node import Node

import tf2_ros
from transforms3d.euler import euler2quat

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import math

# from arduino_driver import ArduinoDriver


class ArduinoDriver:
    def __init__(self, port):
        self.l = 0.16   # расстояние между центрами ведущих колёс в метрах
        self.r = 0.035  # радиус ведущих колёс в метрах
        self.ser = serial.Serial(port, 9600, timeout=0.1)
        sleep(3)

    def _write(self, command):
        self.ser.write((command + '\n').encode())

    def _read_arduino(self):
        try:
            return self.ser.readline().decode().strip()
        except:
            return None

    def proceed_command(self, twist):
        v = twist.linear.x      # м/с
        w = twist.angular.z     # рад/с

        v_left  = v - w * self.l / 2
        v_right = v + w * self.l / 2

        left_cmd  = int(v_left  * 100)
        right_cmd = int(v_right * 100)

        self._write(f"N {left_cmd} {right_cmd}")

    def get_robot_data(self):
        self._write("g")

        gy25 = None
        enc = None
        voltage = None

        start = time()
        while time() - start < 0.3:
            line = self._read_arduino()
            if not line:
                continue

            if line.startswith("ENC:"):
                enc = [int(x) for x in line.split()[1:]]
            elif line.startswith("VOLTAGE:"):
                voltage = [float(x) for x in line.split()[1:]]
            elif line.startswith("GY25:"):
                gy25 = [int(x) for x in line.split()[1:]]

            if enc is not None:
                break

        return gy25, enc, voltage

class RobotDriver(Node):
    def __init__(self):
        super().__init__("robot_driver")

        # железо
        self.arduino = ArduinoDriver(
            "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
        )

        # ROS интерфейсы
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = TransformBroadcaster(self)

        # состояние
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # таймер 20 Гц
        self.create_timer(0.05, self.update)

    def cmd_callback(self, msg):
        self.arduino.proceed_command(msg)

    def update(self):
        gy25, enc, voltage = self.arduino.get_robot_data()
        if enc is None:
            return

        # примитивная одометрия по энкодерам
        dl = enc[0] / 1000.0
        dr = enc[1] / 1000.0
        ds = (dl + dr) / 2.0
        dyaw = (dr - dl) / self.arduino.l

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        self.yaw += dyaw
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        # публикуем odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)

        self.odom_pub.publish(odom)

        # публикуем TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w

        self.tf_pub.sendTransform(t)


def main():
    rclpy.init()
    node = RobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

