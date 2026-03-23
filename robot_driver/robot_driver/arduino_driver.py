from time import time, sleep
import serial


class ArduinoDriver:
    def __init__(self, port):
        # изменить self.l, если будет погрешность при повороте
        self.l = 0.16  # Расстояние между колесами (метры)
        self.r = 0.0325  # Радиус колес (6.5см / 2 в метрах)
        try:
            self.ser = serial.Serial(port, 9600, timeout=0.05)
            sleep(2)  # Пауза для перезагрузки Arduino
        except Exception as e:
            print(f"Ошибка подключения к Serial: {e}")

    def _write(self, command):
        try:
            self.ser.write((command + '\n').encode())
        except:
            pass

    def _read_arduino(self):
        try:
            if self.ser.in_waiting > 0:
                return self.ser.readline().decode().strip()
        except:
            return None
        return None

    def proceed_command(self, twist):
        # Прошивка ждет команду 'N' и значения в см/с
        v = twist.linear.x * 100.0  # м/с -> см/с
        w = twist.angular.z  # рад/с

        v_left = int(v - w * (self.l * 100) / 2.0)
        v_right = int(v + w * (self.l * 100) / 2.0)

        self._write(f"N {v_left} {v_right}")

    def get_robot_data(self):
        self._write("g")

        enc = None
        voltage = None

        start_t = time()
        while time() - start_t < 0.1:
            line = self._read_arduino()
            if not line: continue

            if line.startswith("ENC:"):
                try:
                    enc = [int(x) for x in line.split()[1:]]
                except:
                    continue
            elif line.startswith("VOLTAGE:"):
                try:
                    voltage = [float(x) for x in line.split()[1:]]
                except:
                    continue

            if enc is not None: break

        return None, enc, voltage