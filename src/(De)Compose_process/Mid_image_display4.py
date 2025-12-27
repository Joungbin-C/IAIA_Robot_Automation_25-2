#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from indy_driver.msg import process_info  # ✅ process_info 메시지 사용

class CellDetectorNode:
    def __init__(self):
        rospy.init_node('cell_detector', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber
        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.action, queue_size=1)

        # Publisher
        self.pub_flag = rospy.Publisher("process_info", process_info, queue_size=1)

        # --- Params --- #
        self.resize_n = 600
        self.enable_viz = True
        self.crop_x = (150, 400)      # crop 범위 (x)
        self.crop_y = (120, 400)      # crop 범위 (y)
        self.after_crop_resize = 500  # ✅ crop 후 다시 resize 크기 (정방형)

        # HSV threshold for orange color
        # ⚠️ 조명에 따라 조정 필요
        self.lower_orange = np.array([0, 100, 150]) 
        self.upper_orange = np.array([40, 255, 255])
        
        rospy.loginfo("✅ [cell_detector] initialized (orange detection with combine/decompose flag)")

    def action(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"[cell_detector] CvBridgeError: {e}")
            return

        # 1) 이미지 리사이즈
        resized = cv2.resize(cv_image, (self.resize_n, self.resize_n))
        x0, x1 = self.crop_x
        y0, y1 = self.crop_y
        cropped = resized[y0:y1, x0:x1]

        # 3) Resize cropped area
        resized_crop = cv2.resize(cropped, (self.after_crop_resize, self.after_crop_resize))

        # 2) BGR → HSV 변환
        hsv = cv2.cvtColor(resized_crop, cv2.COLOR_BGR2HSV)

        # 3) 주황색 범위 마스크 생성
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        # 4) 노이즈 제거
        morp = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        morp = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((5,5), np.uint8))

        # 5) 컨투어 탐색
        contours, _ = cv2.findContours(morp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 6) 주황색 검출 여부 판단
        msg = process_info()
        if contours:
            # 가장 큰 영역 선택
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:  # 최소 면적 필터 (필요시 조정)
                msg.flag = 0  # decompose mode
                rospy.loginfo(f"🟧 Orange detected → Decompose mode (flag=0, area={area:.1f})")
                if self.enable_viz:
                    cv2.drawContours(resized_crop, [largest], -1, (0,255,0), 2)
            else:
                msg.flag = 1  # combine mode
                rospy.loginfo("⚫ Small contour → combine mode (flag=0)")
        else:
            msg.flag = 1  # combine mode
            rospy.loginfo("⚫ No orange detected → combine mode (flag=1)")

        # 7) 메시지 퍼블리시
        self.pub_flag.publish(msg)

        # 8) 시각화
        if self.enable_viz:
            cv2.imshow("Image_Display4 : Orange Detection", resized_crop)
            cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CellDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
