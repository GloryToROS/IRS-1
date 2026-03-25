#!/usr/bin/python3

import serial
import time
import math
import sys

PORT_GYRO = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
PORT_ARDUINO = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'

class Gyroscope:
    def __init__(self):
        self.serial = serial.Serial(PORT_GYRO, 115200, timeout=0.01)
        self.window = [0] * 8
        self.yaw_offset = 0.0
        self.current_yaw = 0.0

    def get_data(self):
        if self.serial.in_waiting > 0:
            chunk = self.serial.read(self.serial.in_waiting)
            for byte in chunk:
                self.window.pop(0)
                self.window.append(byte)
                if self.window[0] == 0xAA and self.window[7] == 0x55:
                    raw = (self.window[1] << 8) | self.window[2]
                    if raw > 32767: raw -= 65536
                    self.current_yaw = (raw / 100.0) - self.yaw_offset

class SmartRobot:
    def __init__(self):
        self.gyro = Gyroscope()
        self.ser_arduino = serial.Serial(PORT_ARDUINO, 9600, timeout=0.1)

        self.gyro_window = [0] * 8
        self.yaw_offset = 0.0
        self.current_yaw = 0.0

        self.x, self.y = 0.0, 0.0
        self.enc_offset_l = 0
        self.enc_offset_r = 0
        self.last_enc_l, self.last_enc_r = None, None

        self.ticks_per_rev = 1300.0
        self.r = 0.0325

    def get_raw_enc(self):
        self.ser_arduino.reset_input_buffer()
        for _ in range(50):
            line = self.ser_arduino.readline().decode(errors='ignore').strip()
            if line.startswith("ENC:"):
                try:
                    parts = line.split()
                    return int(parts[1]), int(parts[2])
                except: continue
            time.sleep(0.01)
        return None

    def reset_encoders(self):
        print("[System] Обнуление энкодеров...")
        res = self.get_raw_enc()
        if res:
            self.enc_offset_l, self.enc_offset_r = res
            self.last_enc_l, self.last_enc_r = 0, 0
            self.x, self.y = 0.0, 0.0
            print(f"[System] База энкодеров установлена: L:{self.enc_offset_l} R:{self.enc_offset_r}")
        else:
            print("[Error] Данные ENC не получены!")

    def reset_coordinates(self):
        """Полный сброс локальных координат и базовых тиков энкодеров"""
        res = self.get_raw_enc()
        if res:
            self.enc_offset_l, self.enc_offset_r = res
            self.last_enc_l, self.last_enc_r = 0, 0
            self.x, self.y = 0.0, 0.0
            return True
        return False

    def update_sensors(self):
        # Гироскоп
        self.gyro.get_data()

        # Ардуино
        if self.ser_arduino.in_waiting > 0:
            line = self.ser_arduino.readline().decode(errors='ignore').strip()
            if line.startswith("ENC:"):
                try:
                    parts = line.split()
                    curr_l = int(parts[1]) - self.enc_offset_l
                    curr_r = int(parts[2]) - self.enc_offset_r

                    if self.last_enc_l is not None:
                        d_l = curr_l - self.last_enc_l
                        d_r = curr_r - self.last_enc_r
                        step_dist = ((d_l + d_r) / 2.0 / self.ticks_per_rev) * (2 * math.pi * self.r)

                        rad = math.radians(self.current_yaw)
                        self.x += step_dist * math.cos(rad)
                        self.y += step_dist * math.sin(rad)

                    self.last_enc_l, self.last_enc_r = curr_l, curr_r
                except: pass

    def drive_straight(self, target_dist_m, speed_cm_s=15):
        self.reset_coordinates()
        self.update_sensors()
        start_x, start_y = self.x, self.y
        start_time = time.time()
        target_yaw = self.current_yaw
        kp = 0.8

        # Оперируем направлением
        direction = 1 if target_dist_m >= 0 else -1

        print(f"[Drive] Старт движения ({'вперед' if direction > 0 else 'назад'}). "
              f"Цель: {target_dist_m}м, Курс: {target_yaw:.1f}°")

        try:
            while True:
                self.update_sensors()

                elapsed = time.time() - start_time
                # Считаем пройденный путь (всегда положительный из-за корня)
                traveled = math.sqrt((self.x - start_x) ** 2 + (self.y - start_y) ** 2)

                # Сравниваем пройденное расстояние с модулем цели
                if traveled >= abs(target_dist_m):
                    self.ser_arduino.write(b"N 0 0\n")
                    print(f"[Drive] Цель достигнута. Итоговое расстояние: {traveled:.4f}м")
                    break

                # PID-коррекция курса
                error = target_yaw - self.current_yaw
                if error > 180: error -= 360
                if error < -180: error += 360

                # При движении назад ошибка инвертируется для правильного подруливания
                correction = error * kp * direction

                # Базовая скорость теперь зависит от знака дистанции
                base_speed = speed_cm_s * direction

                v_l = int(base_speed - correction)
                v_r = int(base_speed + correction)

                self.ser_arduino.write(f"N {v_l} {v_r}\n".encode())

                time.sleep(0.1)
        except KeyboardInterrupt:
            self.ser_arduino.write(b"N 0 0\n")
            print("\n[Drive] Остановлено пользователем")

    def calibrate(self):
        print("[System] Инициализация...")
        self.reset_encoders()
        print("[System] Калибровка гироскопа (2 сек)...")
        time.sleep(2)
        self.update_sensors()
        raw = (self.gyro_window[1] << 8) | self.gyro_window[2]
        if raw > 32767: raw -= 65536
        self.yaw_offset = raw / 100.0
        print(f"[System] Готов.Yaw Offset: {self.yaw_offset:.2f}")

    def brushes_on(self):
        print("[Brushes] Включение щеток...")
        self.ser_arduino.write(b"W\n")

    def brushes_off(self):
        print("[Brushes] Выключение щеток...")
        self.ser_arduino.write(b"w\n")

    def turn_relative(self, angle_deg, speed=15):
        """
        Поворот на месте на заданный угол относительно текущего.
        angle_deg: положительный (налево/против часовой), отрицательный (направо/по часовой)
        """
        self.update_sensors()
        target_yaw = self.current_yaw + angle_deg

        # Нормализуем цель, чтобы она оставалась в пределах [-180, 180]
        if target_yaw > 180: target_yaw -= 360
        if target_yaw < -180: target_yaw += 360

        print(f"[Turn] Текущий: {self.current_yaw:.1f}°, Цель: {target_yaw:.1f}°")

        kp_turn = 0.6  # Коэффициент замедления при приближении к углу
        min_speed = 8  # Минимальная скорость, чтобы моторы не встали из-за трения

        try:
            while True:
                self.update_sensors()

                # Считаем кратчайшую ошибку угла
                error = target_yaw - self.current_yaw
                if error > 180: error -= 360
                if error < -180: error += 360

                # Если подошли достаточно близко (порог 1.5 градуса)
                if abs(error) < 1.5:
                    self.ser_arduino.write(b"N 0 0\n")
                    print(f"[Turn] Поворот завершен. Итоговый угол: {self.current_yaw:.1f}°")
                    break

                # Рассчитываем скорость вращения
                v = error * kp_turn

                # Ограничиваем скорость и не даем ей упасть ниже порога трогания
                side = 1 if v > 0 else -1
                v = side * max(min(abs(v), speed), min_speed)

                # Для разворота на месте: левое минус v, правое плюс v
                v_l = int(-v)
                v_r = int(v)

                self.ser_arduino.write(f"N {v_l} {v_r}\n".encode())

                sys.stdout.write(f"\rПоворот: Error {error:6.2f}° | Команда: N {v_l} {v_r}")
                sys.stdout.flush()

                time.sleep(0.05)
        except KeyboardInterrupt:
            self.ser_arduino.write(b"N 0 0\n")

if __name__ == "__main__":
    robot = SmartRobot()
    robot.calibrate()

    robot.drive_straight(3.60, 30)
    robot.brushes_on()
    robot.drive_straight(0.15, 30)

    time.sleep(1.5)
    robot.drive_straight(-0.1, 30)
    time.sleep(0.2)
    robot.turn_relative(-85)
    time.sleep(1)
    robot.drive_straight(0.6, 30)
    time.sleep(0.2)

    robot.turn_relative(90)
    time.sleep(0.2)
    robot.drive_straight(1.0, 30)
    time.sleep(0.2)
    robot.turn_relative(85)
    time.sleep(0.2)
    robot.drive_straight(0.6, 30)
    time.sleep(1)


    robot.drive_straight(-1.0, 30)
    # time.sleep(1)
    robot.brushes_off()

    # robot.turn_relative(90)
    # time.sleep(0.2)
    # robot.drive_straight(1.0, 30)
    # time.sleep(0.2)
    #
    # robot.drive_straight(0.6, 30)
    # time.sleep(0.2)
    # robot.turn_relative(90)
    # time.sleep(0.2)
    # robot.drive_straight(0.4, 30)
    #
    # # robot.drive_straight(-3.90, 30)
    # # time.sleep(0.2)
    # # robot.brushes_on()
    # # robot.drive_straight(0.8, 30)
    # # robot.brushes_off()

    # robot.brushes_on()
    # time.sleep(10)
    # robot.brushes_off()