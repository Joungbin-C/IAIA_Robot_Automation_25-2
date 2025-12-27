#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from indy_driver.msg import decomp_wind_info


class CellDetectorNode:
    def __init__(self):
        rospy.init_node('cell_detector', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber
        self.sub_image = rospy.Subscriber("camera/image_raw", Image, self.action, queue_size=1)

        # Publisher
        self.pub_cell = rospy.Publisher("decomp_wind_info", decomp_wind_info, queue_size=1)

        # --- Params --- #
        self.resize_n = 600
        self.crop_x = (110, 510)
        self.crop_y = (210, 520)
        self.num_div_x = 3      # 가로 3칸
        self.num_div_y = 2      # 세로 2칸
        self.enable_viz = True

        # 🔥 주황색 범위 설정 (HSV)
        self.lower_orange = np.array([0,  80,  80])
        self.upper_orange = np.array([40, 255, 255])


        # 최소 주황색 픽셀 비율 (ROI 대비 %)
        self.orange_ratio_threshold = 0.10   # 2% 이상이면 셀 안에 물체 있다고 판단

        rospy.loginfo("✅ [cell_detector] initialized (orange-based detection)")

    def action(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"[cell_detector] CvBridgeError: {e}")
            return

        # --- Resize → HSV --- #
        n = self.resize_n
        resized = cv2.resize(cv_image, (n, n))
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

        # --- Crop --- #
        x_start, x_end = self.crop_x
        y_start, y_end = self.crop_y
        cropped_hsv = hsv[y_start:y_end, x_start:x_end]
        vis = resized[y_start:y_end, x_start:x_end].copy()

        # --- Grid 크기 --- #
        step_x = (x_end - x_start) // self.num_div_x
        step_y = (y_end - y_start) // self.num_div_y

        detected_cell = -1
        cell_num = 1

        # --- 셀별 주황색 픽셀 검사 --- #
        for row in range(self.num_div_y):
            for col in range(self.num_div_x):

                # ROI
                x1, y1 = col * step_x, row * step_y
                x2, y2 = x1 + step_x, y1 + step_y
                roi_hsv = cropped_hsv[y1:y2, x1:x2]

                # 주황색 마스크
                mask = cv2.inRange(roi_hsv, self.lower_orange, self.upper_orange)
                orange_pixels = np.sum(mask > 0)
                total_pixels = mask.size
                ratio = orange_pixels / total_pixels

                state = "O" if ratio < self.orange_ratio_threshold else "E"

                # 텍스트 표시
                text = f"{cell_num}:{ratio*100:.1f}%"
                color = (0, 165, 255) if state == "O" else (255, 255, 255)
                cv2.putText(vis, text, (x1 + 10, y1 + 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # 가장 첫 번째로 주황색 감지된 셀 선택
                if state == "O" and detected_cell == -1:
                    detected_cell = cell_num

                cell_num += 1

        # --- 퍼블리시 --- #
        msg = decomp_wind_info()
        msg.cell_num = int(detected_cell if detected_cell != -1 else 0)
        self.pub_cell.publish(msg)

        if detected_cell == -1:
            rospy.loginfo("[cell_detector] publish: cell=0 (no orange detected)")
        else:
            rospy.loginfo(f"[cell_detector] publish: cell={msg.cell_num} (orange detected)")

        # --- 시각화 --- #
        if self.enable_viz:
            # Grid 라인
            for i in range(1, self.num_div_x):
                cv2.line(vis, (i * step_x, 0), (i * step_x, vis.shape[0]), (0, 255, 0), 2)
            for j in range(1, self.num_div_y):
                cv2.line(vis, (0, j * step_y), (vis.shape[1], j * step_y), (0, 255, 0), 2)

            cv2.imshow("Image Display6:", vis)
            cv2.waitKey(1)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CellDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
