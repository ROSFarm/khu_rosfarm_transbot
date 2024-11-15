import os
import sys
from pathlib import Path
import time
import torch
import cv2
import requests
from io import BytesIO
from PIL import Image
import rospy
from std_msgs.msg import Int32
import numpy as np

# YOLOv5 모델 관련 경로 설정
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

# YOLOv5 관련 모듈 불러오기
from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.torch_utils import select_device

# ROS 노드 초기화 및 퍼블리셔 설정
rospy.init_node('apple_counter_node', anonymous=True)
pub = rospy.Publisher('/apple_count', Int32, queue_size=10)

# YOLO 모델 설정
device = select_device('0')
model = DetectMultiBackend(ROOT / 'last.pt', device=device, fp16=True)
stride, names = model.stride, model.names
imgsz = check_img_size((640, 480), s=stride)

# Jetson Nano 1에서 제공하는 HTTP 스트리밍 주소 설정
stream_url = "http://192.168.35.175:5000/video_feed"  # Jetson Nano 1의 IP 주소를 입력

def detect_apples():
    global apple_count, previous_apple_count

    # 사과 개수 변수 초기화
    apple_count = 0
    previous_apple_count = apple_count

    while not rospy.is_shutdown():
        try:
            # HTTP 스트리밍에서 이미지 프레임 가져오기
            response = requests.get(stream_url, stream=True)
            response.raise_for_status()
            for chunk in response.iter_content(chunk_size=1024):
                if not chunk:
                    break
                
                # 스트림에서 이미지 읽기
                img_stream = BytesIO(chunk)
                img = Image.open(img_stream)
                frame = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

                # YOLO 모델 추론 준비
                img = torch.from_numpy(frame).to(device)
                img = img.permute(2, 0, 1).unsqueeze(0)  # (H, W, C) -> (1, C, H, W)
                img = img.half() if model.fp16 else img.float()
                img /= 255  # 0-255 -> 0.0-1.0

                # 모델 추론
                pred = model(img)
                pred = non_max_suppression(pred, 0.5, 0.6, classes=None, max_det=100)

                # 사과 감지 및 개수 세기
                current_apple_count = 0
                for det in pred:
                    if len(det):
                        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()
                        for *xyxy, conf, cls in det:
                            if int(cls) == 0:  # 사과 클래스가 0이라고 가정
                                current_apple_count += 1
                                cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
                                cv2.putText(frame, f"Apple", (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 터미널에 실시간 사과 개수 출력
                print(f"Current detected apples: {current_apple_count}")

                # ROS 토픽으로 사과 개수 발행
                if current_apple_count != previous_apple_count:
                    apple_count = current_apple_count
                    previous_apple_count = apple_count
                    pub.publish(apple_count)

        except requests.exceptions.RequestException as e:
            print("Error: Unable to connect to the video stream")
            break

if __name__ == "__main__":
    detect_apples()

