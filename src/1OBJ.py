import cv2
import numpy as np
from ultralytics import YOLO
import random
import time
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ConnectionException

ip_address = '192.168.4.20'
client = ModbusTcpClient(ip_address)

# Đường dẫn đến mô hình YOLO
model_path = "ttest.engine"
object_detect = YOLO(model_path, task='detect')

# Tên các lớp và màu sắc
class_names = {0: "OK", 1: "NG", 2: "Object3", 3: "Object4", 4: "Object5"}
color_map = {0: (0, 255, 0), 1: (0, 0, 255), 2: (0, 0, 255), 3: (255, 255, 0), 4: (0, 255, 255)}

# Biến điều khiển
detecting = True
show_window = True

def open_camera():
    global cap
    gst_str = (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int)640, height=(int)640, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
    )
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("❌ Không thể kết nối CSI camera!")
        exit()

open_camera()

cv2.namedWindow('Control Panel', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Control Panel', 300, 100)
cv2.imshow('Control Panel', np.zeros((100, 300, 3), dtype=np.uint8))

while True:
    if detecting:
        success, frame = cap.read()
        if not success:
            print("❌ Lỗi đọc frame từ camera!")
            break
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame_resized = cv2.resize(frame, (640, 640))
        results = object_detect(frame_resized, conf=0.5, imgsz=640)
        detections = results[0].boxes

        obj = False
        best_detection = None
        max_conf = 0

        for box, cls, conf in zip(detections.xyxy.int().tolist(), detections.cls.tolist(), detections.conf.tolist()):
            if conf > max_conf:
                max_conf = conf
                best_detection = (box, cls, conf)

        if best_detection:
            x1, y1, x2, y2 = best_detection[0]
            cls = best_detection[1]
            conf = best_detection[2]
            color = color_map.get(int(cls), (random.randint(0,255), random.randint(0,255), random.randint(0,255)))
            label = f"{class_names[int(cls)]}: {conf:.2f}"
            cv2.rectangle(frame_resized, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame_resized, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            if int(cls) == 0:
                obj = True

        if show_window:
            cv2.imshow("Object Detection", frame_resized)
        else:
            try:
                cv2.destroyWindow("Object Detection")
            except:
                pass
    
    M200, M201, M202 = 0, 0, 0
    if client.connect():
        M200_bit = client.read_coils(8, 1)
        M201_bit = client.read_coils(9, 1)
        M202_bit = client.read_coils(10, 1)
        M200 = M200_bit.bits[0] if not M200_bit.isError() else 0
        M201 = M201_bit.bits[0] if not M201_bit.isError() else 0
        if obj and M201 == 1:
            client.write_coils(10, True)
            M202 = M202_bit.bits[0]
            print("Gửi 10")
        else:
            client.write_coils(10, False)
            M202 = 0
    else:
        print("⚠️ Mất kết nối với PLC!")
    
    key = cv2.waitKey(1) & 0xFF
    if key in [ord('q'), ord('Q')]:
        print("🛑 Thoát chương trình...")
        break
    elif not detecting:
        open_camera()
        detecting = True
        print("📸 Camera và phát hiện đối tượng BẬT")
    elif key in [ord('r'), ord('R')]:
        if detecting:
            print("📴 Camera và phát hiện đối tượng TẮT")
            time.sleep(1)
            detecting = False
            cap.release()
    elif key in [ord('w'), ord('W')]:
        show_window = not show_window
        print(f"🔧 Trạng thái hiển thị: {'BẬT' if show_window else 'TẮT'}")

cap.release()
cv2.destroyAllWindows()
