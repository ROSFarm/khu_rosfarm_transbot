#!/usr/bin/env python
# coding:utf-8
import numpy as np
import os
from common import *
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, Int32  # Int32 추가
from dynamic_reconfigure.server import Server
from transbot_laser.cfg import laserAvoidPIDConfig
import subprocess

class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)
        self.linear = 0.3
        self.angular = 1
        self.ResponseDist = 0.3
        self.LaserAngle = 60  # 10~180
        self.Moving = False
        self.switch = False
        self.running = False
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.initial_motion_done = False
        self.sonar_stop = False  # 초음파 센서와 적외선 토픽을 기반으로 로봇 움직임을 멈추는 플래그
        self.infrared_stop = False
        self.infrared_count = 0  # 적외선 센서 수신 횟수 카운트
        self.stop_movement = False
        self.apple_count = 0  # apple_count 초기화
        self.ros_ctrl = ROSCtrl()

        # ROS 구독자 설정
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan)
        self.sub_sonar = rospy.Subscriber('/sonar_dist', Bool, self.sonar_callback)  # 초음파 센서 구독 추가
        self.sub_infrared = rospy.Subscriber('/infrared_sensor', Bool, self.infrared_callback)  # 적외선 토픽 구독 추가
        self.sub_apple_count = rospy.Subscriber('apple_count', Int32, self.apple_count_callback)  # apple_count 토픽 구독 추가

        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)

        # 초기 움직임 시작
        self.initial_motion()

    def apple_count_callback(self, msg):
        # apple_count 토픽 수신 시 콜백 함수
        self.apple_count += msg.data
        bot.set_beep(100)
        sleep(1)
        rospy.loginfo("Received total apple count: {}".format(self.apple_count))

    def initial_motion(self):
        # 초기 전진 동작
        rospy.loginfo("Starting initial forward motion...")
        twist = Twist()
        twist.linear.x = 0.3  # 전진 속도
        self.ros_ctrl.pub_vel.publish(twist)
        sleep(5)  # 5초 동안 전진
        self.ros_ctrl.pub_vel.publish(Twist())  # 정지
        rospy.loginfo("Initial forward motion complete.")
        self.initial_motion_done = True  # 초기 움직임 완료로 설정

    def cancel(self):
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        self.sub_sonar.unregister()
        self.sub_infrared.unregister()
        self.sub_apple_count.unregister()
        rospy.loginfo("Shutting down this node.")

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        return config

    def sonar_callback(self, msg):
        self.sonar_stop = msg.data
        self.update_stop_movement()

    def infrared_callback(self, msg):
        if msg.data:
            self.infrared_count += 1
            rospy.loginfo("Infrared count: {}".format(self.infrared_count))
            self.infrared_stop = True
            self.update_stop_movement()
            sleep(3)
            self.infrared_stop = False
            self.update_stop_movement()

            if self.infrared_count == 4:
                rospy.loginfo("Infrared sensor triggered 3 times. Launching transbot_navigation.launch...")
                subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "roslaunch aruco_move arucomove.launch; exec bash"])
                self.infrared_count = 0

    def registerScan(self, scan_data):
        if not self.initial_motion_done:
            return
        if self.running == True or self.stop_movement == True:
            return
        ranges = np.array(scan_data.ranges)
        sortedIndices = np.argsort(ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0

        for i in sortedIndices:
            if len(ranges) == 720:
                if 20 < i < self.LaserAngle * 2 and ranges[i] < self.ResponseDist:
                    self.Left_warning += 1
                elif (720 - self.LaserAngle * 2) < i < 700 and ranges[i] < self.ResponseDist:
                    self.Right_warning += 1
                elif (700 <= i or i <= 20) and ranges[i] <= self.ResponseDist:
                    self.front_warning += 1
            elif len(ranges) == 360:
                if 10 < i < self.LaserAngle and ranges[i] < self.ResponseDist:
                    self.Left_warning += 1
                elif (350 - self.LaserAngle) < i < 350 and ranges[i] < self.ResponseDist:
                    self.Right_warning += 1
                elif (350 <= i <= 360 or 0 <= i <= 10) and ranges[i] < self.ResponseDist:
                    self.front_warning += 1

    def update_stop_movement(self):
        self.stop_movement = self.sonar_stop or self.infrared_stop
        if self.stop_movement:
            self.stop_robot()
            rospy.loginfo("Robot is stopped due to sensor detection")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.ros_ctrl.pub_vel.publish(twist)

    def robot_move(self):
        while not rospy.is_shutdown():
            if not self.initial_motion_done:
                continue
            if self.ros_ctrl.Joy_active or self.switch or self.stop_movement:
                if self.stop_movement:
                    rospy.loginfo("Robot is stopped due to sensor detection.")
                if self.Moving:
                    self.ros_ctrl.pub_vel.publish(Twist())
                    self.Moving = False
                continue

            self.Moving = True
            twist = Twist()
            if self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning > 10:
                twist.linear.x = -0.15
                twist.angular.z = -self.angular
                self.ros_ctrl.pub_vel.publish(twist)
                sleep(0.2)
            elif self.front_warning > 10 and self.Left_warning <= 10 and self.Right_warning > 10:
                twist.linear.x = 0
                twist.angular.z = self.angular
                self.ros_ctrl.pub_vel.publish(twist)
                sleep(0.2)
            elif self.front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
                twist.linear.x = self.linear
                twist.angular.z = 0
                self.ros_ctrl.pub_vel.publish(twist)
            self.r.sleep()

if __name__ == '__main__':
    rospy.init_node('laser_Avoidance', anonymous=False)
    os.system("rosnode kill /LineDetect")
    tracker = laserAvoid()
    tracker.robot_move()
    rospy.spin()
    tracker.cancel()