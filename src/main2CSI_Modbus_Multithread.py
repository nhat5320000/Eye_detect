import cv2
import numpy as np
import threading
import time
from ultralytics import YOLO
from pymodbus.client import ModbusTcpClient

# Khởi tạo biến toàn cục
frame = None
obj = False
detecting = True
show_window = True
ip_address = '192.168.4.20'
client = ModbusTcpClient(ip_address)

# Khởi tạo mô hình YOLO
model_path = "ttest.engine"
object_detect = YOLO(model_path, task='detect')

# Tên các lớp và màu sắc
class_names = {0: "OK", 1: "NG", 2: "Object3", 3: "Object4", 4: "Object5"}
color_map = {0: (0, 255, 0), 1: (0, 0, 255), 2: (0, 0, 255), 3: (255, 255, 0), 4: (0, 255, 255)}

# Hàm đọc camera liên tục
def read_camera():
    global frame, detecting
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
    
    while detecting:
        success, frame_temp = cap.read()
        if success:
            frame = cv2.rotate(cv2.resize(frame_temp, (640, 640)), cv2.ROTATE_180)
        time.sleep(0.02)  # Đảm bảo vòng lặp không tiêu tốn quá nhiều CPU
    
    cap.release()

# Hàm chạy YOLO liên tục
def detect_objects():
    global obj
    while detecting:
        if frame is not None:
            #frame_resized = frame.copy()  # Tạo bản sao để vẽ lên
            results = object_detect(frame, conf=0.5, imgsz=640)
            obj = 0 in results[0].boxes.cls.tolist()
            # Vẽ bounding boxes
                #for box, cls, conf in zip(detections.xyxy.int().tolist(), detections.cls.tolist(), detections.conf.tolist()):
                #x1, y1, x2, y2 = box
                #color = color_map.get(int(cls), (random.randint(0,255), random.randint(0,255), random.randint(0,255)))
                #label = f"{class_names[int(cls)]}: {conf:.2f}"
                #cv2.rectangle(frame_resized, (x1, y1), (x2, y2), color, 2)
                #cv2.putText(frame_resized, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        #time.sleep(0.05)  # Giảm tải CPU
            # Kiểm tra nếu có phát hiện object
            #for r in results:
                #boxes = r.boxes.xyxy.cpu().numpy()  # Lấy tọa độ (x1, y1, x2, y2)
                #classes = r.boxes.cls.cpu().numpy()  # Lấy nhãn class
                #scores = r.boxes.conf.cpu().numpy()  # Lấy độ tự tin

                #for box, cls, conf in zip(boxes, classes, scores):
                    #x1, y1, x2, y2 = map(int, box)
                    #color = color_map.get(int(cls), (255, 255, 255))  # Mặc định màu trắng nếu không có trong map
                    #label = f"{class_names.get(int(cls), 'Unknown')}: {conf:.2f}"

                    # Vẽ bounding box và label
                    #cv2.rectangle(frame_resized, (x1, y1), (x2, y2), color, 2)
                    #cv2.putText(frame_resized, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Cập nhật frame để hiển thị
            #frame = frame_resized

# Hàm giao tiếp PLC
def plc_communication():
    global obj
    while detecting:
        if client.connect():
            M201_bit = client.read_coils(9, 1)
            M201 = M201_bit.bits[0] if not M201_bit.isError() else 0
            
            if obj and M201 == 1:
                client.write_coils(10, True)
            else:
                client.write_coils(10, False)
        time.sleep(0.1)  # Giảm tần suất gửi tín hiệu

# Chạy các luồng song song
thread_camera = threading.Thread(target=read_camera, daemon=True)
thread_yolo = threading.Thread(target=detect_objects, daemon=True)
thread_plc = threading.Thread(target=plc_communication, daemon=True)

thread_camera.start()
thread_yolo.start()
thread_plc.start()

# Vòng lặp chính để xử lý phím điều khiển
while True:
    if frame is not None and show_window:
        cv2.imshow("Object Detection", frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("🛑 Thoát chương trình...")
        detecting = False
        break
    elif key == ord('r'):
        print("📴 Camera và phát hiện đối tượng TẮT")
        detecting = False
    elif key == ord('w'):
        show_window = not show_window
        if not show_window:
            cv2.destroyWindow("Object Detection")
        print(f"🔧 Trạng thái hiển thị: {'BẬT' if show_window else 'TẮT'}")

cv2.destroyAllWindows()
