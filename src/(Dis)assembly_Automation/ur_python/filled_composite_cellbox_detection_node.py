#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class AssemblyGridDetectionNode: 
    def __init__(self):
        rospy.init_node('assembly_grid_detection_node', anonymous=True)
        self.bridge = CvBridge()
        
        # ==========================================================
        # 📌 퍼블리셔 및 서브스크라이버
        # ==========================================================
        self.detection_pub = rospy.Publisher('/inverse_grid_detection_result', String, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        # === 그리드 설정 ===
        self.P1, self.P2, self.P3 = (105,77), (547,73), (105,433)
        self.cols, self.rows = 4, 3
        self.active_cell_priority = ["1-1", "1-3", "2-2", "2-4", "3-1", "3-3"]
        self.active_cells = set((int(c.split('-')[0]), int(c.split('-')[1])) for c in self.active_cell_priority)
        self.gray_history = [] 

        # === 🟠 주황색 감지 파라미터 ===
        # B, G, R 채널 기준 (0~255)
        self.ORANGE_B_MIN, self.ORANGE_B_MAX = 65, 185
        self.ORANGE_G_MIN, self.ORANGE_G_MAX = 110, 220
        self.ORANGE_R_MIN, self.ORANGE_R_MAX = 220, 255
        self.orange_pixel_ratio = 0.05  # 셀 내 주황색이 5% 이상일 때 감지

    # --------------------------------------------------------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            result_frame = self.orange_area_detect(frame)
            cv2.imshow("Assembly Grid Detection (Orange Area)", result_frame)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Detection Error: {e}")

    # --------------------------------------------------------------
    def orange_area_detect(self, frame):
        display_frame = frame.copy()
        
        xs = np.linspace(self.P1[0], self.P2[0], self.cols+1)
        ys = np.linspace(self.P1[1], self.P3[1], self.rows+1)
        detected_cell_names = set()

        # 1️⃣ 셀별 주황색 영역 감지
        for i in range(self.rows):
            for j in range(self.cols):
                r, c = i+1, j+1
                cell_name = f"{r}-{c}"
                x1, x2 = int(xs[j]), int(xs[j+1])
                y1, y2 = int(ys[i]), int(ys[i+1])

                if y2 <= y1 or x2 <= x1:
                    continue

                cell_roi = frame[y1:y2, x1:x2]
                cell_area = cell_roi.shape[0] * cell_roi.shape[1]

                # 주황색 범위 마스크 (BGR)
                b_min, b_max = self.ORANGE_B_MIN, self.ORANGE_B_MAX
                g_min, g_max = self.ORANGE_G_MIN, self.ORANGE_G_MAX
                r_min, r_max = self.ORANGE_R_MIN, self.ORANGE_R_MAX

                orange_mask = cv2.inRange(cell_roi, (b_min, g_min, r_min), (b_max, g_max, r_max))
                orange_count = np.sum(orange_mask > 0)
                current_ratio = orange_count / cell_area if cell_area > 0 else 0

                color = (80,80,80)
                text = f"{r}-{c} ({current_ratio:.2f})"

                if (r, c) in self.active_cells:
                    color = (0,255,0)

                    if current_ratio >= self.orange_pixel_ratio:
                        detected_cell_names.add(cell_name)
                        color = (0,140,255)  # 🟠 주황색 감지시 색상 강조
                        text = f"DETECTED ({current_ratio:.2f})"

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                    cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                    cv2.putText(display_frame, text, (cx - 60, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                else:
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 1)

        # 2️⃣ 3프레임 안정화 로직 (EMA처럼 프레임별 안정화)
        self.gray_history.append(frozenset(detected_cell_names))
        if len(self.gray_history) > 3:
            self.gray_history.pop(0)

        if len(self.gray_history) == 3 and all(h == self.gray_history[0] for h in self.gray_history):
            stable_detected_set = self.gray_history[0]
            result_str = ""

            if len(stable_detected_set) > 0:
                highest_priority_cell = ""
                for cell_name in self.active_cell_priority:
                    if cell_name in stable_detected_set:
                        highest_priority_cell = cell_name
                        break

                if highest_priority_cell:
                    result_str = highest_priority_cell
                    self.detection_pub.publish(String(result_str))
                    rospy.loginfo(f"✨ Assembly Grid STABLE PRIORITY Detection Published: {result_str}")
                else:
                    result_str = ""
                    self.detection_pub.publish(String(result_str))
                    rospy.logwarn(f"⚠️ No active priority cell found ({result_str})")
            else:
                result_str = ""
                self.detection_pub.publish(String(result_str))
                rospy.loginfo(f"⚠️ Nothing Detected ({result_str})")

            self.gray_history.clear()

        return display_frame


# --------------------------------------------------------------
if __name__ == '__main__':
    try:
        node = AssemblyGridDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
