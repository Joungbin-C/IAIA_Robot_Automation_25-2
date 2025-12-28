#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image               # sensor_msg 패키지로부터 Image type을 import함
from cv_bridge import CvBridge, CvBridgeError   # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능

class CameraNode:

    def __init__(self):
        self.bridge = CvBridge()                # cv_bridge 객체 생성
        self.image_pub = rospy.Publisher("camera/image_raw",Image,queue_size=1)    # "camera/image_raw"라는 토픽으로 메시지를 publish할 publisher 객체 생성
        self.cap = cv2.VideoCapture(2)          # 카메라 연결을 위한 VideoCapture 객체 생성

    def run(self):
        rospy.init_node('camera_node', anonymous=True)  # 노드 이름 "camera_node"로 초기화
        rate = rospy.Rate(30)                           # 루프 실행 주기 : 30hz
        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안
            ret, frame = self.cap.read()                # 카메라로부터 이미지를 읽음
            if ret:                                     # 이미지가 정상적으로 읽혀진 경우
                try:
                    # 읽어들인 이미지를 ROS Image 메시지로 변환하여 토픽으로 publish
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                
                except CvBridgeError as e:
                    print(e)                            # CvBridge 변환 예외 처리
            rate.sleep()                                # 지정된 루프 실행 주기에 따라 대기

if __name__ == '__main__':
    try:
        camera = CameraNode()       # CameraNode 객체 생성
        camera.run()                # run 메서드 실행
    except rospy.ROSInterruptException:
        pass
