import zenoh
import cv2
import numpy as np
import time
from ultralytics import YOLO

print("[Brain] Загрузка нейросети в RTX 5080...")
# Сюда впиши свой 'best.pt', если он есть. Либо оставь 'yolov8n.pt' для автозагрузки дефолтной.
model = YOLO('best.pt') 
model.to('cuda') # Принудительно отправляем в GPU

print("[Brain] Подключаюсь к сети робота...")
conf = zenoh.Config()
conf.insert_json5("mode", '"client"')
conf.insert_json5("connect/endpoints", '["tcp/10.42.0.1:7447"]')
session = zenoh.open(conf)

latest_frame = None

def camera_cb(sample):
    global latest_frame
    payload = sample.payload.to_bytes()
    np_arr = np.frombuffer(payload, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    if frame is not None:
        latest_frame = frame

print("[Brain] Слушаю топик 'robot/sensors/camera'...")
sub = session.declare_subscriber("robot/sensors/camera", camera_cb)

frame_count = 0
start_time = time.time()

try:
    while True:
        if latest_frame is not None:
            # Забираем кадр
            frame = latest_frame.copy()
            latest_frame = None 
            
            # --- МАГИЯ YOLO ---
            # verbose=False чтобы не спамить в консоль на каждый кадр
            results = model(frame, verbose=False) 
            
            # Получаем кадр с уже нарисованными рамками (bounding boxes)
            display_frame = results[0].plot()
            # ------------------
            
            # Увеличиваем для удобства
            display_frame = cv2.resize(display_frame, (640, 480))
            
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed
            if frame_count > 30: 
                frame_count = 0
                start_time = time.time()
                
            cv2.putText(display_frame, f"System FPS: {fps:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        
            cv2.imshow("RTX 5080 - YOLO Inference", display_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n[Brain] Отключение...")
finally:
    cv2.destroyAllWindows()
    session.close()
