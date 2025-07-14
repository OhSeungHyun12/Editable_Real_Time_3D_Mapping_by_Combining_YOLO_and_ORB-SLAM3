import cv2
import time
import datetime
import torch
from ultralytics import YOLO
import os

start = time.time()

def log_timing(label):
    sec = time.time() - start
    formatted = str(datetime.timedelta(seconds=sec)).split(".")[0]
    print(f"[{label:<30}] ⏱ {formatted}")

# CUDA 확인
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("CUDA is available:", torch.cuda.is_available())
print(f"Using device: {device}")
log_timing("CUDA / Device Init")

# 📸 비디오 장치 탐색 (리눅스 전용: /dev/video*)
def list_video_devices():
    devices = []
    for i in range(50):  # 최대 10개까지 확인
        dev_path = f"/dev/video{i}"
        if os.path.exists(dev_path):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    devices.append(i)
                cap.release()
    return devices

log_timing("Video Device Discovery")
available_devices = list_video_devices()
print("Available video devices:", available_devices)

if not available_devices:
    print("❌ No camera devices found.")
    exit()

# 사용자 입력
try:
    x = int(input(f"Select camera number from {available_devices}: "))
    if x not in available_devices:
        raise ValueError
except Exception:
    print("❌ Invalid camera selection")
    exit()

# 모델 로드
model = YOLO("LAB_best.pt")  # 또는 yolov11.pt
log_timing("Model Loaded")

# 웹캠 연결
capture = cv2.VideoCapture(x)
if not capture.isOpened():
    print("❌ Can't open webcam.")
    exit()

# 해상도 설정
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

frame_count = 0
prev_time = time.time()
start_time = prev_time
log_timing("Camera Activated")

while True:
    ret, frame = capture.read()
    if not ret:
        print("⚠️ Cannot read Frame.")
        break

    curr_time = time.time()
    term = curr_time - prev_time
    fps = 1.0 / term if term > 0 else 0
    prev_time = curr_time
    frame_count += 1

    # YOLOv11 감지
    results = model(frame)
    annotated_frame = results[0].plot()

    # 정보 표시
    fps_info = f"term = {term:.3f}s, FPS = {fps:.2f}"
    frame_info = f"Frame: {frame_count}"
    cv2.putText(annotated_frame, fps_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
    cv2.putText(annotated_frame, frame_info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("YOLOv11 Webcam", annotated_frame)
    log_timing("Live Uptime")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료
end_time = time.time()
elapsed_time = end_time - start_time
avg_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
print(f"✅ Total frames: {frame_count}, Avg FPS: {avg_fps:.2f}")

capture.release()
cv2.destroyAllWindows()
log_timing("Total Runtime")
