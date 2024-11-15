import argparse
import os
import sys
from pathlib import Path
import time
from Transbot_Lib import Transbot
bot = Transbot()
import torch
import torch.backends.cudnn as cudnn
import cv2
import rospy  # ROS 추가
from std_msgs.msg import Int32  # ROS 메시지 타입 추가

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device

from deep_sort_pytorch.utils.parser import get_config
from deep_sort_pytorch.deep_sort import DeepSort
from graphs import bbox_rel, draw_boxes

@torch.no_grad()
def run(
        weights=ROOT / 'last.pt',  # model.pt path(s)
        source='source.txt',  # astra camera - video3(right), video4(left)
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 480),  # inference size (height, width)
        conf_thres=0.5,  # confidence threshold
        iou_thres=0.6,  # NMS IOU threshold
        max_det=100,  # maximum detections per image
        device='0',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        line_thickness=3,  # bounding box thickness (pixels)
        half=True,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        config_deepsort="deep_sort_pytorch/configs/deep_sort.yaml"  # Deep Sort configuration
):
    # ROS 노드 초기화 및 퍼블리셔 설정
    rospy.init_node('red_num_node', anonymous=True)

    source = str(source)
    webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
        bs = len(dataset)  # batch_size

    # Initialize multiple DeepSort instances for each stream
    deepsorts = []
    for _ in range(bs):
        cfg = get_config()
        cfg.merge_from_file(config_deepsort)
        deepsort_instance = DeepSort(cfg.DEEPSORT.REID_CKPT,
                                     max_dist=cfg.DEEPSORT.MAX_DIST, min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                                     nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                                     max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                                     use_cuda=True)
        deepsorts.append(deepsort_instance)


    #사과 개수 변수
    red_num = 0
    previous_red_num = red_num  # 이전 red_num을 추적
    previous_positions = [{} for _ in range(bs)]

    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    print('START!')   
    
    for path, im, im0s, vid_cap, s in dataset:
        im = torch.from_numpy(im).to(device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # Inference
        pred = model(im, augment=augment)

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        

        for i, det in enumerate(pred):  # detections per image (stream i)
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count #p가 카메라 번호!
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh

            if len(det):
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                bbox_xywh = []
                confs = []
                for *xyxy, conf, cls in det:
                    x_c, y_c, bbox_w, bbox_h = bbox_rel(*xyxy)
                    bbox_xywh.append([x_c, y_c, bbox_w, bbox_h])
                    confs.append([conf.item()])

                xywhs = torch.Tensor(bbox_xywh)
                confss = torch.Tensor(confs)

                outputs = deepsorts[i].update(xywhs, confss, im0)

                if len(outputs) > 0:
                    bbox_xyxy = outputs[:, :4]
                    identities = outputs[:, -1]
                    '''for j, (x1, y1, x2, y2) in enumerate(bbox_xyxy):
                          # 빨간색 바운딩 박스와 ID 출력
                        cv2.rectangle(im0, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                        cv2.putText(im0, f'ID: {identities[j]}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)'''

                    for j, identity in enumerate(identities):
                        x1, y1, x2, y2 = bbox_xyxy[j]
                        center_x = int((x1 + x2) / 2)  # 객체 바운딩 박스의 중심 x 좌표

                        if identity in previous_positions[i]:
                            previous_center_x = previous_positions[i][identity]
                            
                            if i == 1 and previous_center_x > 320 and center_x < 320:
                                red_num += 1
                                print(f"Camera 1: red_num incremented to {red_num} for ID {identity}")
                            elif i == 0 and previous_center_x < 320 and center_x > 320:
                                red_num += 1
                                print(f"Camera 0: red_num incremented to {red_num} for ID {identity}")

                        previous_positions[i][identity] = center_x

            else:
                deepsorts[i].increment_ages()

            # red_num이 증가할 때만 발행
            if red_num > previous_red_num:
                previous_red_num = red_num  # 이전 red_num 업데이트
                on_time = 100
                bot.set_beep(on_time)
                time.sleep(1)

            '''if view_img:
                cv2.line(im0, (320, 0), (320, 480), (255, 0, 0), 2)
                cv2.imshow(str(p), im0)''' 
                

if __name__ == "__main__":
    run()
