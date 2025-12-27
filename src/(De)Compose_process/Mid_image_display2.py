#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from indy_driver.msg import bracket_info

class CellDetectorNode:
    def __init__(self):
        rospy.init_node('cell_detector', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber
        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.action, queue_size=1)

        # Publisher
        self.pub_place = rospy.Publisher("bracket_info", bracket_info, queue_size=1)

        # --- Params --- #
        self.resize_n = 600
        self.crop_x = (150, 400)
        self.crop_y = (120, 400)
        self.after_crop_resize = 1000
        self.enable_viz = True
    
        # Threshold params
        self.blur_ksize = 3
        self.bin_thresh = 100
        self.bin_maxval = 255
        self.bin_type = cv2.THRESH_BINARY
        self.thresh1 = 200
        self.thresh2 = 250

        # Area filter
        self.min_area = 100000
        self.max_area = 300000

        # Real size(cm)
        self.real_width_cm = 7.9
        self.real_height_cm = 6.8

        # Intensity threshold
        self.intensity_threshold = 100000000 # Example threshold for intensity sum

        rospy.loginfo("✅ [cell_detector] initialized (no stabilization; immediate publish)")

    def publish_no_place(self):
        msg = bracket_info()
        msg.center_x = np.float32(np.nan)
        msg.center_y = np.float32(np.nan)
        self.pub_place.publish(msg)
        rospy.logwarn("[place_detector] NO_PLACE")

    def action(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"[cell_detector] CvBridgeError: {e}")
            return

        # 1) 원본 리사이즈
        resized = cv2.resize(cv_image, (self.resize_n, self.resize_n))

        # 2) crop
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        x0, x1 = self.crop_x
        y0, y1 = self.crop_y
        crop_gray = gray[y0:y1, x0:x1]

        # 3) 다시 리사이즈
        resize_gray = cv2.resize(crop_gray, (self.after_crop_resize, self.after_crop_resize))

        # 4) intensity 계산
        intensity_sum = np.sum(resize_gray)


        # 5) intensity 총합이 기준을 넘으면 임의의 x, y 값 전송
        if intensity_sum < self.intensity_threshold:
            x_random = np.random.uniform(-0.1, 0.1)  # 예시: x값은 -0.1과 0.1 사이의 랜덤 값
            y_random = np.random.uniform(-0.1, 0.1)  # 예시: y값은 -0.1과 0.1 사이의 랜덤 값

            msg = bracket_info()
            msg.center_x = float(x_random)
            msg.center_y = float(y_random)
            self.pub_place.publish(msg)
            rospy.loginfo(f"✅ Publish ({msg.center_x:.4f}, {msg.center_y:.4f})")
        else:
            self.publish_no_place()

        # 6) 시각화
        if self.enable_viz:
            vis = cv2.cvtColor(resize_gray, cv2.COLOR_GRAY2BGR)
            cv2.imshow("Image Display2 : Braket Arrive Checking", vis)
            cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CellDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

