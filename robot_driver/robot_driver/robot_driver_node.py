import serial
from time import sleep, time
import math
import time
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy


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
        v = twist.linear.x
        w = twist.angular.z

        v_left  = v - w * self.l / 2
        v_right = v + w * self.l / 2

        omega_left  = v_left  / self.r
        omega_right = v_right / self.r

        left_cmd  = int(omega_left  * 100)  # см/с
        right_cmd = int(omega_right * 100)  # см/с

        self._write(f"N {left_cmd} {right_cmd}")

    def get_robot_data(self):
        self._write("g")
        gy25 = enc = voltage = None

        start = time()
        while time() - start < 0.5:
            line = self.read_arduino()
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