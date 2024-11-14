#!/usr/bin/env python3
#coding=utf-8
import rospy
from std_msgs.msg import Bool
from Transbot_Lib import Transbot

# Transbot 객체 생성
bot = Transbot()

# LED 제어 함수
def colorful_lamps(r, g, b):
    bot.set_colorful_lamps(0xff, r, g, b)

# 노드 종료 시 LED 끄기
def shutdown_hook():
    rospy.loginfo("Shutting down node, turning off LED.")
    colorful_lamps(0, 0, 0)  # LED 끄기

# 초음파 데이터 콜백 함수
def sonar_callback(data):
    global last_received_time
    rospy.loginfo(f"Current distance: {data.data} cm")
    last_received_time = rospy.get_time()  # 마지막으로 메시지를 받은 시간 업데이트

    # 거리가 15cm 이하일 때만 LED 작동
    if data.data:
        colorful_lamps(255, 0, 0)  # 빨간색 LED를 계속 켬
    else:
        colorful_lamps(0, 0, 0)  # 거리가 15cm 초과하면 LED 끄기

# 주기적으로 토픽 수신 여부 확인 및 LED 끄기
def check_sonar_timeout(event):
    current_time = rospy.get_time()
    if current_time - last_received_time > 1.0:  # 1초 동안 데이터 수신이 없으면
        colorful_lamps(0, 0, 0)  # LED 끄기
        rospy.loginfo("No sonar data received for 1 second. Turning off LED.")

if __name__ == "__main__":
    rospy.init_node('led_controller')

    # 노드 종료 시 shutdown_hook 실행
    rospy.on_shutdown(shutdown_hook)

    # 마지막 수신 시간 초기화
    last_received_time = rospy.get_time()

    # 초음파 센서에서 발행하는 토픽 구독 (tcp_nodelay 설정 추가)
    rospy.Subscriber('/sonar_dist', Bool, sonar_callback, queue_size=10)

    # 주기적으로 토픽 수신 여부를 확인하는 타이머 (1초마다 실행)
    rospy.Timer(rospy.Duration(1), check_sonar_timeout)

    # ROS 노드가 종료되지 않도록 유지
    rospy.spin()

