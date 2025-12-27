#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from indy_driver.msg import comp_pose_info

class CellDetectorNode:
    def __init__(self):
        rospy.init_node('cell_detector', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber
        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.action, queue_size=1)

        # Publisher
        self.pub_place = rospy.Publisher("comp_pose_info", comp_pose_info, queue_size=1)

        # --- Params --- #
        self.resize_n = 600
        self.enable_viz = True
        self.crop_x = (220, 380)      # crop 범위 (x)
        self.crop_y = (120, 380)      # crop 범위 (y)
        self.after_crop_resize = 500  # ✅ crop 후 다시 resize 크기 (정방형)

        # HSV threshold for orange color
        # ⚠️ 조명에 따라 조정 필요
        self.lower_orange = np.array([0, 80, 150]) 
        self.upper_orange = np.array([40, 255, 255])
        
        # --- Threshold & Edge Params --- #
        self.blur_ksize = 3
        self.bin_thresh = 100
        self.bin_maxval = 255
        self.bin_type = cv2.THRESH_BINARY
        self.thresh1 = 200
        self.thresh2 = 250

        # --- Area Filter --- #
        self.min_area = 10000
        self.max_area = 240000

        # --- 실제 크기 (cm) --- #
        self.real_width_cm  =  5.6
        self.real_height_cm =  7.0

        rospy.loginfo("✅ [cell_detector] initialized (crop→resize→mm publish, x↔y swapped)")

    def publish_no_place(self):
        msg = comp_pose_info()
        msg.center_x = np.float32(np.nan)
        msg.center_y = np.float32(np.nan)
        self.pub_place.publish(msg)
        rospy.logwarn("[place_detector] published → NO_PLACE (center_x/center_y = NaN)")


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
        crop_h, crop_w = resized_crop.shape[:2]
        cx_img = crop_h / 2.0
        cy_img = crop_w / 2.0

        # 2) BGR → HSV 변환
        hsv = cv2.cvtColor(resized_crop, cv2.COLOR_BGR2HSV)

        # 3) 주황색 범위 마스크 생성
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        # 4) 노이즈 제거
        morp = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        morp = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((5,5), np.uint8))

        contours, _ = cv2.findContours(morp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        kept = 0
        centers = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_area <= area  :
                cv2.drawContours(resized_crop, [cnt], -1, (0, 255, 0), 2)
                kept += 1
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy))
                    cv2.circle(resized_crop, (cx, cy), 4, (0, 255, 255), -1)


        # --- 5️⃣ 가장 큰 contour 중심좌표 계산 및 mm 변환 --- #
        if len(centers) > 0:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m01"] / M["m00"])
                cy = int(M["m10"] / M["m00"])

                # 중심 기준 픽셀 오프셋
                dx_pix = cx - cx_img
                dy_pix = cy - cy_img

                # 픽셀 → cm 비율 (리사이즈된 기준)
                x_scale = self.real_height_cm / self.after_crop_resize
                y_scale = self.real_width_cm / self.after_crop_resize

                # 픽셀 → cm
                x_cm = -dx_pix * x_scale
                y_cm = -dy_pix * y_scale


                # cm → m
                x_m = x_cm * 0.01
                y_m = y_cm * 0.01


                # ✅ x↔y 교체 후 publish
                msg = comp_pose_info()
                msg.center_x = float(x_m) + 0.108
                msg.center_y = float(y_m) + 0.0055
                self.pub_place.publish(msg)

                rospy.loginfo(f"[place_detector] published → X={msg.center_x:.4f} m, Y={msg.center_y:.4f} m")
            else:
                self.publish_no_place()
        else:
            self.publish_no_place()

        # --- 6️⃣ 시각화 --- #
        if self.enable_viz:
            h, w = resized_crop.shape[:2]
            cx, cy = int(w / 2), int(h / 2)
            cv2.line(resized_crop, (cx - 50, cy), (cx + 50, cy), (0, 0, 255), 2)  
            cv2.line(resized_crop, (cx, cy - 50), (cx, cy + 50), (0, 0, 255), 2)  

            cv2.imshow("Image_Display3 : Cell pose (x,y)", resized_crop)
            cv2.waitKey(1)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CellDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
