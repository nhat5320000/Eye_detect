import cv2
from ultralytics import YOLO

# Load YOLO model
eye_detect = YOLO("training/eyes-detect/yolov11n/yolov11_last.engine", task='detect')

# Mở camera
cap = cv2.VideoCapture(0)
if not cap or not cap.isOpened():
    print("❌ Không thể mở camera!")
    exit()

while True:
    success, frame = cap.read()
    if not success:
        print("❌ Không thể đọc frame!")
        break

    # Resize ảnh về đúng kích thước model yêu cầu (320x320)
    frame_resized = cv2.resize(frame, (320, 320))

    # Phát hiện mắt (đảm bảo imgsz=320 để tránh lỗi)
    detections = eye_detect(frame_resized, conf=0.7, imgsz=320)[0].boxes.xyxy.int().tolist()
    
    if detections:
        print("OK")  # Nếu tìm thấy mắt, in "OK"
cap.release()
