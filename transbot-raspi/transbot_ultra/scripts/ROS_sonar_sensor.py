#!/usr/bin/env python3

import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import Bool

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class Sonar:
    def __init__(self):
        rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher('/sonar_dist', Bool, queue_size=1)
        self.r = rospy.Rate(1)

    def publish_status(self, status):
        data = Bool()
        data.data = status
        self.distance_publisher.publish(data)

gpio.setmode(gpio.BCM)
trig = 23
echo = 24

gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)

sensor = Sonar()
time.sleep(0.5)
print('-----------------------------------------------------------------sonar start')

try:
    while True:
        gpio.output(trig, False)
        time.sleep(0.1)
        gpio.output(trig, True)
        time.sleep(0.00001)
        gpio.output(trig, False)
        
        while gpio.input(echo) == 0:
            pulse_start = time.time()
        while gpio.input(echo) == 1:
            pulse_end = time.time()
            
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17000
        
        if pulse_duration >= 0.01746:
            continue
        elif distance > 300 or distance == 0:
            continue
        
        distance = round(distance, 3)
        
        # 15cm 이하일 때 True, 그렇지 않으면 False로 토픽 발행
        sensor.publish_status(distance <= 15)
        
        sensor.r.sleep()
        
except (KeyboardInterrupt, SystemExit):
    gpio.cleanup()
    sys.exit(0)
except:
    gpio.cleanup()
