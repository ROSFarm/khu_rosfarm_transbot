#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32MultiArray  # x, y, z 좌표를 전송할 메시지 타입
import os

script_dir = os.path.dirname(os.path.realpath(__file__))  # 스크립트의 디렉토리 가져오기
calibration_file_path = os.path.join(script_dir, 'camera_calibration.npz')

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=2  # 180도 회전
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, publisher):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()
   
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(f"ids: {ids}")
    if ids is None:
        coordinates = Float32MultiArray(data=[10.0, 0.0, 10.0])  
        publisher.publish(coordinates)
        print("not found")
        
    if ids is not None:

        for i in range(len(ids)):
            # 아루코 마커의 자세 추정
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            print(f"rotation vector: {rvec}")
            print(f"translation vector: {tvec}\n")
           
            # 변환된 위치를 화면에 출력
            x, y, z = tvec[0][0] * 100  # cm로 변환
            text = f"id: {ids[i][0]} x: {x:.2f}, y: {y:.2f}, z: {z:.2f}"
            cv2.putText(frame, text, (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
           
            # AR 태그 ID가 특정 값일 때
            target_id = 10  # 찾고자 하는 AR 태그 ID 설정
            if ids[i][0] == target_id:
                print("Hi, I'm here")
                print(f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")  # ID 10인 경우 x, y, z 출력

                # x, y, z 좌표를 퍼블리셔를 통해 발행
                coordinates = Float32MultiArray(data=[x, y, z])
                publisher.publish(coordinates)

            # 마커를 화면에 표시
            cv2.aruco.drawDetectedMarkers(frame, corners)

    
    return frame

if __name__ == '__main__':
    # ROS 노드 초기화 및 퍼블리셔 설정
    rospy.init_node('aruco_detector')
    aruco_publisher = rospy.Publisher('aruco_marker_coordinates', Float32MultiArray, queue_size=10)
    
    aruco_dict_type = cv2.aruco.DICT_7X7_250  # 아루코 마커 타입을 7x7로 설정
       
    # 카메라 보정 파일에서 보정 매트릭스와 왜곡 계수를 로드
    with np.load(calibration_file_path) as data:
        calibration_matrix = data['camera_matrix']
        dist_coeffs = data['distortion_coefficients']
       
    # GStreamer 파이프라인을 이용하여 CSI 카메라에서 영상을 받아옵니다.
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

    if video_capture.isOpened():
        try:
            rate = rospy.Rate(20) #10Hz send data
            while not rospy.is_shutdown():
                ret_val, frame = video_capture.read()
                if not ret_val:
                    print("Error: Unable to capture video")
                    break

                # 아루코 마커 자세 추정
                output = pose_estimation(frame, aruco_dict_type, calibration_matrix, dist_coeffs, aruco_publisher)
               
                # 결과를 화면에 표시
                cv2.imshow('Estimated Pose', output)
               
                # 'q' 키를 누르면 종료
                if cv2.waitKey(1) == ord('q'):
                    break
                    
                rate.sleep() #Hz regulate
                
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")
