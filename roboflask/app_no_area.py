from flask import Flask, render_template
import rospy
from std_msgs.msg import Int32

# Flask 앱 초기화
app = Flask(__name__)

# 전역 변수로 사과 개수를 저장
apple_count = 0

# ROS 콜백 함수
def apple_callback(data):
    global apple_count
    apple_count = data.data  # 수신된 사과 개수를 업데이트

# Flask 라우팅
@app.route('/')
def index():
    global apple_count
    return render_template('index.html', apple_count=apple_count)

if __name__ == '__main__':
    # ROS 초기화
    rospy.init_node('apple_listener', anonymous=True)
    
    # apple_num 토픽 구독
    rospy.Subscriber('apple_count', Int32, apple_callback)
    
    # Flask 앱 실행 (백그라운드 스레드로 ROS 스핀)
    app.run(host='0.0.0.0', port=5000)
