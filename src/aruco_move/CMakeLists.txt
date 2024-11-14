cmake_minimum_required(VERSION 3.0.2)
project(aruco_move)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/arsudong.py
  scripts/picamera.py  # 두 번째 Python 파일 이름으로 수정하세요
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

