#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String # 발행할 메시지 타입

class GrayDetectionNode:
    def __init__(self):
        rospy.init_node('gray_detection_node', anonymous=True)
        self.bridge = CvBridge()
        # ==========================================================
        # 📌 토픽 이름
        # ==========================================================
        self.detection_pub = rospy.Publisher('/gray_detection_result', String, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        # === 그리드 설정 ===
        self.P1, self.P2, self.P3 = (105,77), (547,73), (105,433)
        self.cols, self.rows = 4, 3
        # 📌 활성 셀 정의 (순서는 우선순위와 동일): 1-1이 가장 높고, 3-3이 가장 낮습니다.
        self.active_cell_priority = ["1-1", "1-3", "2-2", "2-4", "3-1", "3-3"]
        self.active_cells = set((int(c.split('-')[0]), int(c.split('-')[1])) for c in self.active_cell_priority)
        self.gray_history = [] # 안정화용

        # === 새로운 밝은 픽셀 감지 임계값 ===
        self.bright_intensity_thresh = 215 # RGB 채널의 최소값
        self.bright_pixel_ratio = 0.10     # 감지 최소 면적 비율 (10%)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            result_frame = self.bright_area_detect(frame) 
            cv2.imshow("Bright Area Grid Cell Detection", result_frame)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Detection Error: {e}")

    def bright_area_detect(self, frame):
        display_frame = frame.copy() 
        
        xs = np.linspace(self.P1[0], self.P2[0], self.cols+1)
        ys = np.linspace(self.P1[1], self.P3[1], self.rows+1)
        detected_cell_names = set() # 감지된 셀 이름 (예: "1-1")

        # 1. 셀 별 밝은 영역 감지
        for i in range(self.rows):
            for j in range(self.cols):
                r, c = i+1, j+1
                cell_name = f"{r}-{c}"
                x1, x2 = int(xs[j]), int(xs[j+1])
                y1, y2 = int(ys[i]), int(ys[i+1])
                
                if y2 <= y1 or x2 <= x1: continue

                cell_roi = frame[y1:y2, x1:x2]
                cell_area = cell_roi.shape[0] * cell_roi.shape[1]

                # B, G, R 채널 값이 모두 215 이상인 픽셀 찾기
                bright_mask = np.all(cell_roi >= self.bright_intensity_thresh, axis=2)
                bright_pixel_count = np.sum(bright_mask)
                current_ratio = bright_pixel_count / cell_area if cell_area > 0 else 0
                
                color = (80,80,80) # 기본 회색 (비활성)
                text = f"{r}-{c} ({current_ratio:.2f})"
                
                if (r,c) in self.active_cells:
                    color = (0,255,0) # 초록색 (활성 대기)
                    
                    if current_ratio >= self.bright_pixel_ratio:
                        detected_cell_names.add(cell_name) # 감지된 셀 이름 저장
                        color = (255,165,0) # 주황색 (감지됨)
                        text = f"DETECTED ({current_ratio:.2f})"
                        
                    # 활성 셀 테두리 및 텍스트 표시
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                    cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                    cv2.putText(display_frame, text, (cx - 60, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                else:
                    # 비활성 셀 테두리 표시
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 1)


        # 2. 5프레임 안정화 로직
        self.gray_history.append(frozenset(detected_cell_names))
        if len(self.gray_history) > 5: self.gray_history.pop(0)
        
        # 5프레임이 채워지고, 모든 프레임에서 같은 결과가 나왔을 때
        if len(self.gray_history) == 5 and all(h == self.gray_history[0] for h in self.gray_history):
            
            stable_detected_set = self.gray_history[0]
            result_str = ""

            if len(stable_detected_set) > 0:
                # 📌 3. 감지된 셀 목록에서 우선순위가 가장 높은 셀 하나를 선택
                highest_priority_cell = ""
                for cell_name in self.active_cell_priority:
                    if cell_name in stable_detected_set:
                        highest_priority_cell = cell_name
                        break
                
                if highest_priority_cell:
                    # 💡 우선순위가 높은 셀 하나만 발행
                    result_str = highest_priority_cell
                    self.detection_pub.publish(String(result_str))
                    rospy.loginfo(f"✨ Bright Area STABLE PRIORITY Detection Published: {result_str}")
                else:
                    # (이 경우는 이론상 발생하기 어려움: stable_detected_set이 비어있지 않은데 우선순위 목록에 없음)
                    result_str = "" 
                    self.detection_pub.publish(String(result_str))
                    rospy.logwarn(f"--- ⚠️ Bright Area STABLE: No active priority cell found ({result_str}) ---")
                    
            else:
                # 🚨 아무것도 감지되지 않았을 경우: 빈 문자열 발행
                result_str = "" 
                self.detection_pub.publish(String(result_str))
                rospy.loginfo(f"--- ⚠️ Bright Area STABLE Nothing Detected ({result_str}) ---")
                
            # 발행 후, 히스토리를 초기화하여 다음 새로운 감지를 대기 (중복 발행 방지)
            self.gray_history.clear() 
            
        return display_frame # 플롯된 프레임을 반환

if __name__ == '__main__':
    try:
        node = GrayDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()