import cv2
import numpy as np
from ultralytics import YOLO

# Load mô hình YOLO
eye_detect = YOLO("training/eyes-detect/yolov11n/yolov11_last.engine", task='detect')

# Mở camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Không thể mở camera!")
    exit()

# Tạo cửa sổ điều khiển luôn hiển thị
cv2.namedWindow('Control Panel', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Control Panel', 300, 100)
cv2.imshow('Control Panel', np.zeros((100, 300, 3), dtype=np.uint8))

# Biến điều khiển hiển thị
show_window = False

while True:
    success, frame = cap.read()
    if not success:
        print("❌ Không thể đọc frame!")
        break

    # Xử lý frame
    frame_resized = cv2.resize(frame, (320, 320))
    results = eye_detect(frame_resized, conf=0.11, imgsz=320)
    detections = results[0].boxes.xyxy.int().tolist()

    # Vẽ bounding box
    for box in detections:
        x1, y1, x2, y2 = box
        cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame_resized, "Eye", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Hiển thị cửa sổ kết quả
    if show_window:
        cv2.imshow("Eye Detection", frame_resized)
        # Tự động tắt khi đóng bằng nút X
        if cv2.getWindowProperty("Eye Detection", cv2.WND_PROP_VISIBLE) < 1:
            show_window = False
    else:
        try:
            cv2.destroyWindow("Eye Detection")
        except:
            pass

    # Xử lý sự kiện phím từ cửa sổ Control Panel
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == ord('Q'):
        print("🛑 Thoát chương trình...")
        break
    elif key == ord('w') or key == ord('W'):
        show_window = not show_window
        print(f"🔧 Trạng thái hiển thị: {'BẬT' if show_window else 'TẮT'}")

# Giải phóng tài nguyên
cap.release()
cv2.destroyAllWindows()
