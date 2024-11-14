#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from Transbot_Lib import Transbot
import time

# Transbot 초기화
bot = Transbot()

# 전역 변수로 로봇의 동작 상태 설정
is_moving = False  # 로봇이 현재 움직이는지 여부를 추적
latest_msg = None  # 최신 아루코 마커 좌표 메시지 저장

def finish(fin):
    """동작 완료 시 비프음을 발생시키는 함수."""
    if fin:
        bot.set_beep(100)
        time.sleep(0.5)
        bot.set_beep(100)
        time.sleep(0.5)
        bot.set_beep(300)
        time.sleep(5)

def car_motion(line, angular, duration):
    """
    로봇을 특정 속도로 움직이는 함수.
    
    :param line: 선형 속도 (0~100)
    :param angular: 각속도 (0~100)
    :param duration: 동작 시간 (초)
    """
    global is_moving  # 전역 변수 사용
    speed_l = line / 100.0  # 선형 속도 변환
    speed_a = angular / 100.0  # 각속도 변환
    bot.set_car_motion(speed_l, speed_a)  # 동작 신호
    is_moving = True  # 동작 중 상태로 변경
    time.sleep(duration)  # 동작 시간 동안 대기
    bot.set_car_motion(0, 0)  # 정지
    is_moving = False  # 동작 완료 상태로 변경

def marker_callback(msg):
    global is_moving, latest_msg

    # 로봇이 움직이는 중이라면 메시지를 무시하고 최신 메시지만 저장합니다.
    if is_moving:
        latest_msg = msg  # 최신 메시지를 저장
        print("Currently moving, storing marker coordinates for later processing.")
        return

    # 아루코 마커 좌표 수신 (센티미터 단위)
    x, y, z = msg.data
    print(f"Received marker coordinates in cm: x={x}, y={y}, z={z}")

    # 좌표에 따라 로봇 동작 결정
    if abs(x) > 3:  # 중앙에서 3cm 이상 차이가 나면 회전
        if x < 0:
            car_motion(3, 10, 2)  # 좌회전
        else:
            car_motion(3, -10, 2)  # 우회전
    else:
        car_motion(10, 0, 1)  # 전진

    # z 값에 따라 특정 거리 내에 도달 시 정지
    if z < 9:  # z 값이 8cm 이하일 경우
        bot.set_car_motion(0, 0)  # 멈춤
        print("Marker reached")
        finish(True)

def main():
    global is_moving, latest_msg
    # ROS 노드 초기화
    rospy.init_node('arsudong', anonymous=True)

    # 아루코 마커 좌표 토픽 구독
    rospy.Subscriber('aruco_marker_coordinates', Float32MultiArray, marker_callback, queue_size=1)

    # 주기 설정
    rate = rospy.Rate(20)  # 10Hz

    try:
        while not rospy.is_shutdown():
            # 동작이 완료된 후 최신 좌표 메시지를 처리
            if not is_moving and latest_msg is not None:
                marker_callback(latest_msg)  # 저장된 최신 메시지 처리
                latest_msg = None  # 처리 후 메시지 초기화
            
            rate.sleep()  # 루프 주기에 따라 대기
    except rospy.ROSInterruptException:
        print("Node interrupted.")
    finally:
        bot.set_car_motion(0, 0)  # 프로그램 종료 시 모터 정지

if __name__ == '__main__':
    main()
