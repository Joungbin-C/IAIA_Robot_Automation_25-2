#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, cv2, numpy as np, os, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

class StableGrayDetectionNode:
    def __init__(self):
        rospy.init_node('stable_gray2_detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.detection_pub = rospy.Publisher('/grid2_fallback/result', Point, queue_size=1) 
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # ==========================================================
        # 🧩 사용자 조정 파라미터 (Hough에 맞게 조정)
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

        # 반지름 필터 (픽셀)
        self.MIN_RADIUS = 170
        self.MAX_RADIUS = 250

        # 중심 허용 거리 (mm)
        self.CENTER_TOL_MM = 20.0

        # Gray 톤 필터링 범위
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

        rospy.loginfo("⚪ Stable Gray Detection Node initialized (Hough Transform, Limited ROI)")

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
                    best_circle_data = (x, y, r)

        if best_circle_data is not None:
            x, y, r = best_circle_data

            # EMA 안정화
            if self.prev_center is None:
                self.prev_center = np.array([x, y], dtype=float)
                self.prev_radius = r
            else:
                self.prev_center = (1 - self.alpha) * self.prev_center + self.alpha * np.array([x, y])
                self.prev_radius = (1 - self.alpha) * self.prev_radius + self.alpha * r

            x, y = self.prev_center
            r = self.prev_radius

            # 픽셀 → mm 변환
            dx_px = float(x - self.CENTER_PX[0])
            dy_px = float(y - self.CENTER_PX[1])
            dx_mm = dx_px * self.scale_x
            dy_mm = dy_px * self.scale_y
            dist_mm = math.sqrt(dx_mm**2 + dy_mm**2)

            # 오프셋 적용
            dx_mm_total = -dy_mm + self.CAMERA_OFFSET_X_MM
            dy_mm_total = -dx_mm + self.CAMERA_OFFSET_Y_MM

            if dist_mm <= self.CENTER_TOL_MM:
                # 시각화
                cv2.circle(output, (int(x), int(y)), int(r), (0,255,0), 2)
                cv2.circle(output, (int(x), int(y)), 5, (0,0,255), -1)
                cv2.circle(output, self.CENTER_PX, 5, (255,0,0), -1)
                
                # --- ✅ 텍스트 출력 (raw + XOFFSET 적용)
                cv2.putText(output, f"dx={dx_mm:.1f} dy={dy_mm:.1f}",
                            (25,45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                cv2.putText(output, f"XOFF -> dx={dx_mm_total:.1f} dy={dy_mm_total:.1f}",
                            (25,85), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

                # 퍼블리시
                self.detection_pub.publish(Point(x=float(dx_mm_total), y=float(dy_mm_total), z=float(r)))
                rospy.loginfo_throttle(1.0, f"📍 Valid Circle: dx={dx_mm_total:.1f}, dy={dy_mm_total:.1f}")
            else:
                rospy.logwarn_throttle(1.0, f"⚠️ Circle ignored (dist={dist_mm:.1f}mm > {self.CENTER_TOL_MM}mm)")
                self._publish_fail()
        else:
            rospy.logwarn_throttle(1.0, "❌ No circle found in Hough area.")
            self._publish_fail()

        # 시각화
        cv2.rectangle(output, (OFFSET, OFFSET), (OFFSET + HOUGH_SIZE, OFFSET + HOUGH_SIZE), (255, 0, 0), 2)
        cv2.imshow("Stable Gray2 Detection (Hough Transform, Limited ROI)", output)
        cv2.imshow("Gray2 Hough Filtered Input", gray_blurred)
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
