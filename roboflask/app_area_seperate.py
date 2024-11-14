from flask import Flask, render_template
import rospy
from std_msgs.msg import Int32

app = Flask(__name__)

# 전역 변수로 각 영역의 사과 개수를 저장
area1_count = 0
area2_count = 0
area3_count = 0

# ROS 콜백 함수
def area1_callback(data):
    global area1_count
    area1_count = data.data  # area1의 사과 개수 업데이트

def area2_callback(data):
    global area2_count
    area2_count = data.data  # area2의 사과 개수 업데이트

def area3_callback(data):
    global area3_count
    area3_count = data.data  # area3의 사과 개수 업데이트

# Flask 라우팅
@app.route('/')
def index():
    global area1_count, area2_count, area3_count
    total_apples = area1_count + area2_count + area3_count
    return render_template(
        'index2.html',
        apple_count=total_apples,
        area1_count=area1_count,
        area2_count=area2_count,
        area3_count=area3_count
    )

if __name__ == '__main__':
    # ROS 초기화
    rospy.init_node('apple_listener', anonymous=True)
    
    # 각각의 토픽 구독
    rospy.Subscriber('area1', Int32, area1_callback)
    rospy.Subscriber('area2', Int32, area2_callback)
    rospy.Subscriber('area3', Int32, area3_callback)
    
    # Flask 앱 실행
    app.run(host='0.0.0.0', port=5000, debug=True)
