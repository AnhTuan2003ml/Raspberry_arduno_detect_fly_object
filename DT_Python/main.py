import cv2
import threading
import numpy as np
from ultralytics import YOLO
import LiquidCrystal_I2C
import time
import queue
import serial
import RPi.GPIO as GPIO

# Khởi tạo màn hình LCD
lcd_screen = LiquidCrystal_I2C.lcd()

lcd_screen.clear()
# Khởi tạo mô hình YOLOv8n (nano)
model = YOLO("best.pt")  # Sử dụng YOLOv8n (nano) để tiết kiệm tài nguyên

# Thiết lập serial communication với Arduino
ser = serial.Serial('/dev/ttyS0', 9600)
time.sleep(2)  # Chờ Arduino khởi động

# Khởi tạo GPIO cho các đèn LED và còi
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
led1_pin = 17
led2_pin = 27
coi_pin = 22
GPIO.setup(led1_pin, GPIO.OUT)
GPIO.setup(led2_pin, GPIO.OUT)
GPIO.setup(coi_pin, GPIO.OUT)

# Hàm cập nhật màn hình LCD
def update_lcd(detections):
    if len(detections) == 0:
        time.sleep(0.3)
        # Không có phát hiện, hiển thị dòng 1: "No Detection"
        lcd_screen.display("No Detection", 1, 0)
        lcd_screen.display("", 2, 0)
        print("No Detection")
        GPIO.output(led1_pin, GPIO.LOW)
        GPIO.output(led2_pin, GPIO.HIGH)
        GPIO.output(coi_pin, GPIO.LOW)
    else:
        
            # Xử lý phát hiện đối tượng từ YOLO
            num_detections = len(detections)  # Số lượng vật thể được phát hiện
            first_detection = detections[0]  # Lấy vật thể đầu tiên được phát hiện
            class_id = int(first_detection.cls[0])  # Lấy ID lớp (class ID) của vật thể
            object_name = model.model.names[class_id]  # Tra cứu tên vật thể từ class ID

            # Hiển thị tên và số lượng vật thể lên dòng 1 của LCD
            display_text = f"{object_name}: {num_detections}"
            lcd_screen.display(display_text, 1, 0)
            print(f"Detected: {display_text}")

            # Kích hoạt đèn LED và còi
            GPIO.output(led1_pin, GPIO.HIGH)
            GPIO.output(led2_pin, GPIO.LOW)
            GPIO.output(coi_pin, GPIO.HIGH)
        

# Xử lý khung hình và cập nhật màn hình LCD
def process_frame(frame):
    result = model(frame, agnostic_nms=True)[0]
    detections = result.boxes  # Lấy các bounding boxes (hộp giới hạn)
    lcd_screen.clear()
    if len(detections) > 0:
        update_lcd(detections)
    else:
        update_lcd([])  # Không có phát hiện

# Hàm xử lý YOLO trong luồng riêng
def yolo_thread(frame_queue):
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            process_frame(frame)

# Hàm chính
def main():
    frame_width, frame_height = 320, 240  # Giảm độ phân giải để giảm tải
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    # Khởi tạo hàng đợi khung hình
    frame_queue = queue.Queue(maxsize=1)

    # Khởi tạo luồng YOLO
    threading.Thread(target=yolo_thread, args=(frame_queue,), daemon=True).start()

    prev_time = 0
    fps_limit = 4  # Xử lý 2 khung hình mỗi giây (giới hạn FPS)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Kiểm tra thời gian để giới hạn FPS
        current_time = time.time()
        if current_time - prev_time > 1 / fps_limit:
            prev_time = current_time

            # Resize để giảm tải cho xử lý
            frame_resized = cv2.resize(frame, (320, 240))

            # Đưa khung hình vào hàng đợi xử lý YOLO nếu rảnh
            if frame_queue.empty():
                frame_queue.put(frame_resized)

            # Hiển thị khung hình lên cửa sổ
            cv2.imshow("YOLOv8", frame_resized)
             # Kiểm tra dữ liệu từ Arduino
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8',errors='ignore').rstrip()
                    if line.isdigit():
                        lcd_screen.display(str(line), 2, 0)  # Hiển thị dòng 2 từ Arduino
                        print(f"Arduino: {line[:2]}")
                    ser.reset_input_buffer()
                except Exception as e:
                    print(f"Error: {e}")
        # Thoát vòng lặp nếu nhấn phím ESC
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
