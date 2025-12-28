#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, cv2, numpy as np, os, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

class StableGrayDetectionNode:
    def __init__(self):
        rospy.init_node('stable_gray_detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.detection_pub = rospy.Publisher('/grid1_fallback/result', Point, queue_size=1) 
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # ==========================================================
        # 🧩 사용자 조정 파라미터
        # ==========================================================
        self.ROI_WIDTH = 400
        self.ROI_HEIGHT = 400
        self.TARGET_SIZE = (1200, 1200)
        self.CENTER_PX = (600, 600)
        self.DETECTION_OFFSET = 300

        # 픽셀 → mm 변환
        self.scale_x = 0.13
        self.scale_y = 0.13

        # 카메라 오프셋
        self.CAMERA_OFFSET_X_MM = 88.0
        self.CAMERA_OFFSET_Y_MM = 3.0

        # 반지름 필터
        self.MIN_RADIUS = 170
        self.MAX_RADIUS = 250
        self.CENTER_TOL_MM = 10.0

        # Gray 톤 필터
        self.LOWER_GRAY = 140
        self.UPPER_GRAY = 255

        # Hough 파라미터
        self.HOUGH_DP = 1
        self.HOUGH_MIN_DIST = 100
        self.HOUGH_PARAM1 = 50
        self.HOUGH_PARAM2 = 20

        # EMA 필터
        self.prev_center = None
        self.prev_radius = None
        self.alpha = 0.35

        # 🟠 주황색 감지 파라미터 (반대 조건)
        self.ORANGE_RATIO_THRESH = 0.03  # 3% 이상이면 무시
        self.ORANGE_MIN = (50, 80, 240)
        self.ORANGE_MAX = (120, 150, 255)

        # 🟡 노란색 감지 파라미터 (너가 지정한 EXACT 값)
        self.YELLOW_RATIO_THRESH = 0.03

        self.lower_yellow1 = np.array([20, 120, 120])
        self.upper_yellow1 = np.array([32, 255, 255])

        self.lower_yellow2 = np.array([0, 0, 0])
        self.upper_yellow2 = np.array([0, 0, 0])

        # ==========================================================
        # 📂 카메라 캘리브레이션
        # ==========================================================
        calib_path = "/home/kiminyeop/catkin_ws/src/ur_python/src/dataset_bra2_checkboard3/circlegrid_calibration_results_8x6_20mm.npz"
        if os.path.exists(calib_path):
            data = np.load(calib_path)
            self.camera_matrix = data["camera_matrix"]
            self.dist_coeffs = data["dist_coeffs"]
            rospy.loginfo(f"📷 Loaded camera calibration from: {calib_path}")
        else:
            rospy.logwarn("⚠️ Calibration file not found — skipping undistortion.")
            self.camera_matrix, self.dist_coeffs = None, None

        rospy.loginfo("⚪ Stable Gray Detection Node initialized (Gray only, ignore Orange+Yellow)")

    # --------------------------------------------------------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.camera_matrix is not None:
                frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

            H, W = frame.shape[:2]
            x1 = int(W/2 - self.ROI_WIDTH/2)
            y1 = int(H/2 - self.ROI_HEIGHT/2)
            cropped = frame[y1:y1+self.ROI_HEIGHT, x1:x1+self.ROI_HEIGHT]
            resized = cv2.resize(cropped, self.TARGET_SIZE, interpolation=cv2.INTER_LINEAR)

            self.detect_gray_circle(resized)

        except Exception as e:
            rospy.logerr(f"Gray Detection Error: {e}")

    # --------------------------------------------------------------
    def detect_gray_circle(self, frame):
        output = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_filtered = cv2.inRange(gray, self.LOWER_GRAY, self.UPPER_GRAY)
        gray_blurred = cv2.GaussianBlur(gray_filtered, (9, 9), 2)

        # === HSV 변환 (노란색 검출을 위해 추가)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        OFFSET = self.DETECTION_OFFSET
        HOUGH_SIZE = 600
        HOUGH_INPUT = gray_blurred[OFFSET:OFFSET + HOUGH_SIZE, OFFSET:OFFSET + HOUGH_SIZE]

        circles = cv2.HoughCircles(
            HOUGH_INPUT, cv2.HOUGH_GRADIENT,
            self.HOUGH_DP, self.HOUGH_MIN_DIST,
            param1=self.HOUGH_PARAM1, param2=self.HOUGH_PARAM2,
            minRadius=self.MIN_RADIUS, maxRadius=self.MAX_RADIUS
        )

        best_circle_data = None
        min_dist_to_center = float('inf')

        # 🔹 가장 중앙에 가까운 원 선택
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                x_rel, y_rel, r = i[0], i[1], i[2]
                x = x_rel + OFFSET
                y = y_rel + OFFSET
                dx_px = float(x - self.CENTER_PX[0])
                dy_px = float(y - self.CENTER_PX[1])
                dist_px = math.sqrt(dx_px**2 + dy_px**2)
                if dist_px < min_dist_to_center:
                    min_dist_to_center = dist_px
                    best_circle_data = (x, y, r, 1.0)

        # 🔸 결과 처리
        if best_circle_data is not None:
            x, y, r, _ = best_circle_data

            # EMA 안정화
            if self.prev_center is None:
                self.prev_center = np.array([x, y], dtype=float)
                self.prev_radius = r
            else:
                self.prev_center = (1 - self.alpha) * self.prev_center + self.alpha * np.array([x, y])
                self.prev_radius = (1 - self.alpha) * self.prev_radius + self.alpha * r

            x, y = self.prev_center
            r = self.prev_radius

            # === 🟠 원 내부 오렌지 비율 계산 ===
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            cv2.circle(mask, (int(x), int(y)), int(r), 255, -1)
            orange_mask = cv2.inRange(frame, self.ORANGE_MIN, self.ORANGE_MAX)
            orange_inside = cv2.bitwise_and(orange_mask, orange_mask, mask=mask)
            orange_ratio = np.sum(orange_inside > 0) / np.sum(mask > 0)
            rospy.loginfo_throttle(1.0, f"🍊 Orange ratio inside circle: {orange_ratio*100:.2f}%")

            # === 🟡 원 내부 노란색 비율 계산 (너가 지정한 HSV 기준 사용) ===
            yellow_mask1 = cv2.inRange(hsv, self.lower_yellow1, self.upper_yellow1)
            yellow_mask2 = cv2.inRange(hsv, self.lower_yellow2, self.upper_yellow2)
            yellow_mask = cv2.bitwise_or(yellow_mask1, yellow_mask2)

            yellow_inside = cv2.bitwise_and(yellow_mask, yellow_mask, mask=mask)
            yellow_ratio = np.sum(yellow_inside > 0) / np.sum(mask > 0)
            rospy.loginfo_throttle(1.0, f"🟡 Yellow ratio inside circle: {yellow_ratio*100:.2f}%")

            # === 픽셀 → mm 변환 ===
            dx_px = float(x - self.CENTER_PX[0])
            dy_px = float(y - self.CENTER_PX[1])
            dx_mm = dx_px * self.scale_x
            dy_mm = dy_px * self.scale_y
            dist_mm = math.sqrt(dx_mm**2 + dy_mm**2)

            # === 카메라 오프셋 보정 ===
            dx_mm_total = -dy_mm + self.CAMERA_OFFSET_X_MM
            dy_mm_total = -dx_mm + self.CAMERA_OFFSET_Y_MM

            # ✅ 회색 원은 감지 + 주황색 OR 노란색 > threshold 이면 무시
            if (
                dist_mm <= self.CENTER_TOL_MM and
                orange_ratio <= self.ORANGE_RATIO_THRESH and
                yellow_ratio <= self.YELLOW_RATIO_THRESH
            ):
                cv2.circle(output, (int(x), int(y)), int(r), (0,255,0), 2)
                cv2.circle(output, (int(x), int(y)), 5, (0,0,255), -1)
                cv2.circle(output, self.CENTER_PX, 5, (255,0,0), -1)

                cv2.putText(output, f"dx={dx_mm:.1f} dy={dy_mm:.1f} (OK gray only)",
                            (25,45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                cv2.putText(output, f"XOFF -> dx={dx_mm_total:.1f} dy={dy_mm_total:.1f}",
                            (25,85), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

                self.detection_pub.publish(Point(x=float(dx_mm_total), y=float(dy_mm_total), z=float(r)))
                rospy.loginfo_throttle(1.0, f"✅ Valid Gray (no orange/yellow): dx={dx_mm_total:.1f}, dy={dy_mm_total:.1f}")

            else:
                rospy.logwarn_throttle(1.0, 
                    f"⚠️ Ignored (dist={dist_mm:.1f}mm, "
                    f"orange={orange_ratio*100:.1f}%, yellow={yellow_ratio*100:.1f}%)")
                self._publish_fail()

        else:
            rospy.logwarn_throttle(1.0, "❌ No circle found.")
            self._publish_fail()

        # 시각화
        cv2.rectangle(output, (OFFSET, OFFSET), (OFFSET + HOUGH_SIZE, OFFSET + HOUGH_SIZE), (255, 0, 0), 2)
        cv2.imshow("Stable Gray Detection (Gray only, Ignore Orange + Yellow)", output)
        cv2.imshow("Hough Filtered Input", gray_blurred)
        cv2.imshow("Orange Mask Inside Circle", orange_inside)
        cv2.imshow("Yellow Mask Inside Circle", yellow_inside)
        cv2.waitKey(1)

    # --------------------------------------------------------------
    def _publish_fail(self):
        self.detection_pub.publish(Point(x=0.0, y=0.0, z=0.0))

# --------------------------------------------------------------
if __name__ == '__main__':
    try:
        StableGrayDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
