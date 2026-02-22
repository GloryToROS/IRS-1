#!/usr/bin/python3

#!/usr/bin/env python3
import serial
import sys

PORT = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'

def main():
    try:
        # Настройка порта аналогично termios в C++
        ser = serial.Serial(
            port=PORT,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
    except Exception as e:
        print(f"Ошибка: Не удалось открыть порт {PORT}: {e}")
        return

    print("--- ДАТЧИК ОЖИЛ (ПИТОН-ВЕРСИЯ) ---")

    # Окно на 8 байт, как массив w[8] в твоем коде
    window = [0] * 8

    try:
        while True:
            # Читаем доступные байты
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)

                for byte in chunk:
                    # Сдвигаем окно (аналог твоего цикла j от 0 до 7)
                    window.pop(0)
                    window.append(byte)

                    # Проверка заголовка и концевика
                    if window[0] == 0xAA and window[7] == 0x55:
                        # Твоя логика: High - window[1], Low - window[2]
                        yaw_h = window[1]
                        yaw_l = window[2]

                        # Собираем signed int16
                        combined = (yaw_h << 8) | yaw_l
                        if combined > 32767:
                            combined -= 65536

                        angle = combined / 100.0

                        # Печать с возвратом каретки
                        sys.stdout.write(f"Угол Yaw: {angle:8.2f}°\r")
                        sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nВыход...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
