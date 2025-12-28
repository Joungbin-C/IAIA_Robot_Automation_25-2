#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, cv2, numpy as np, os, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

class StableRedDetectionNode:
    def __init__(self):
        rospy.init_node('stable_red_detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.detection_pub = rospy.Publisher('/red_detection/result', Point, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # ==========================================================
        # 📂 카메라 캘리브레이션 결과 불러오기
        # ==========================================================
        calib_path = "/home/kiminyeop/catkin_ws/src/ur_python/src/dataset_bra2_checkboard3/circlegrid_calibration_results_8x6_20mm.npz"
        if os.path.exists(calib_path):
            data = np.load(calib_path)
            self.camera_matrix = data["camera_matrix"]
            self.dist_coeffs = data["dist_coeffs"]
            rospy.loginfo(f"📷 Loaded camera calibration from: {calib_path}")
        else:
            rospy.logwarn("⚠️ Calibration file not found — skipping undistortion.")
            self.camera_matrix = None
            self.dist_coeffs = None

        # ==========================================================
        # ROI 및 화면 설정
        # ==========================================================
        self.ROI_WIDTH = 400
        self.ROI_HEIGHT = 400
        self.TARGET_SIZE = (1200, 1200)
        self.CENTER_PX = (600, 600)

        # ==========================================================
        # Red HSV 임계값
        # ==========================================================
        # Yellow detection (Hue 20~32)
        self.lower_red1 = np.array([20, 120, 120])
        self.upper_red1 = np.array([32, 255, 255])

        # Not used for yellow, but placeholder to avoid errors
        self.lower_red2 = np.array([0, 0, 0])
        self.upper_red2 = np.array([0, 0, 0])


        # ==========================================================
        # Morphology 커널
        # ==========================================================
        self.kernel = np.ones((5,5), np.uint8)

        # ==========================================================
        # px → mm 변환 비율 (실험값)
        # ==========================================================
        self.scale_x = 0.1435  # mm/px
        self.scale_y = 0.1425  # mm/px

        # ==========================================================
        # 카메라 오프셋 (Tool0 기준)
        # ==========================================================
        self.CAMERA_OFFSET_X_MM = 89.0   # X축 방향 (mm)
        self.CAMERA_OFFSET_Y_MM = 4.0    # Y축 방향 (mm)

        # ==========================================================
        # 기울기 보정 (deg)
        # ==========================================================
        self.roll_tilt_deg = 0.0   # x축 방향 (좌우)
        self.pitch_tilt_deg = 0.0  # y축 방향 (앞뒤)

        # ==========================================================
        # 지수평활 필터 (EMA)
        # ==========================================================
        self.prev_center = None
        self.prev_radius = None
        self.alpha = 0.35

        rospy.loginfo("🔴 Stable Red Detection Node initialized (Calibrated + 82 mm offset + tilt correction)")

    # --------------------------------------------------------------
    # 🎥 메인 콜백
    # --------------------------------------------------------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # === 📸 왜곡 보정 적용 ===
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

            # === 중앙 ROI 자르기 ===
            H, W = frame.shape[:2]
            x1 = int(W/2 - self.ROI_WIDTH/2)
            y1 = int(H/2 - self.ROI_HEIGHT/2)
            cropped = frame[y1:y1+self.ROI_HEIGHT, x1:x1+self.ROI_WIDTH]

            # === 1200x1200 확대 ===
            resized = cv2.resize(cropped, self.TARGET_SIZE, interpolation=cv2.INTER_LINEAR)

            # === 빨간 원 검출 ===
            self.red_detect(resized)

        except Exception as e:
            rospy.logerr(f"Red Detection Error: {e}")

    # --------------------------------------------------------------
    # 🔴 빨간 원 검출
    # --------------------------------------------------------------
    def red_detect(self, frame):
        # --- 영상 안정화 대기 (1 초) ---

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.GaussianBlur(mask, (9,9), 2)

        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.2, minDist=190,
                                   param1=100, param2=25, minRadius=210, maxRadius=430)
        output = frame.copy()

        if circles is not None:
            circles = np.uint16(np.around(circles))[0]
            x, y, r = max(circles, key=lambda c: c[2])

            # === EMA 안정화 ===
            if self.prev_center is None:
                self.prev_center = np.array([x, y], dtype=float)
                self.prev_radius = r
            else:
                self.prev_center = (1 - self.alpha) * self.prev_center + self.alpha * np.array([x, y])
                self.prev_radius = (1 - self.alpha) * self.prev_radius + self.alpha * r

            x, y = self.prev_center
            r = self.prev_radius

            # === 픽셀 → mm 변환 ===
            dx_px = float(x - self.CENTER_PX[0])
            dy_px = float(y - self.CENTER_PX[1])
            dx_mm = dx_px * self.scale_x
            dy_mm = dy_px * self.scale_y

            # === 기울기 보정 ===
            pitch_rad = math.radians(self.pitch_tilt_deg)
            roll_rad  = math.radians(self.roll_tilt_deg)
            dx_mm_corr = dx_mm / math.cos(pitch_rad)
            dy_mm_corr = dy_mm / math.cos(roll_rad)

            # === 오프셋 적용 (카메라 → Tool 기준) ===
            dx_mm_total = -dy_mm_corr + self.CAMERA_OFFSET_X_MM
            dy_mm_total = -dx_mm_corr + self.CAMERA_OFFSET_Y_MM

            # === 시각화 ===
            cv2.circle(output,(int(x),int(y)),int(r),(0,255,0),2)
            cv2.circle(output,(int(x),int(y)),5,(0,0,255),-1)
            cv2.circle(output,self.CENTER_PX,5,(255,0,0),-1)
            cv2.putText(output, f"dx={dx_mm_total:.1f} mm, dy={dy_mm_total:.1f} mm",
                        (25,45), cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)

            # === 퍼블리시 ===
            point_msg = Point(x=float(dx_mm_total), y=float(dy_mm_total), z=float(r))
            self.detection_pub.publish(point_msg)
            rospy.loginfo(f"📍 Published: dx={dx_mm_total:.1f} mm, dy={dy_mm_total:.1f} mm, r={r:.1f}")
        else:
            self.detection_pub.publish(Point(x=0.0, y=0.0, z=0.0))
            rospy.logwarn("❌ Red circle not found, publishing (0,0,0)")

        cv2.imshow("Stable Red Detection (Tilt-corrected + 82 mm offset)", output)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)


# --------------------------------------------------------------
# 🚀 실행부
# --------------------------------------------------------------
if __name__ == '__main__':
    try:
        StableRedDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
