import cv2
import zenoh
import time
import sys
import json

print("[Edge] Запуск видеостримера...")

# Поднимаем Zenoh (он сам поймет, что он в сети 10.42.0.x)
conf = zenoh.Config()
conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')
session = zenoh.open(conf)
pub_cam = session.declare_publisher("robot/sensors/camera")

# Инициализируем камеру (у тебя вроде 320x240, давай пока оставим так для максимальной скорости)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("[ОШИБКА] Камера не найдена!")
    sys.exit(1)

print("[Edge] Стриминг начался. Кидаю кадры в 'robot/sensors/camera'...")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        # Сжимаем кадр в JPEG (качество 80% — идеальный баланс веса и качества для YOLO)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        
        # Отправляем байты в Zenoh
        pub_cam.put(buffer.tobytes())
        
        # Небольшой sleep, чтобы не спалить проц Малины (даст ~30 FPS)
        time.sleep(0.03)

except KeyboardInterrupt:
    print("\n[Edge] Остановка стримера...")
finally:
    cap.release()
    session.close()
