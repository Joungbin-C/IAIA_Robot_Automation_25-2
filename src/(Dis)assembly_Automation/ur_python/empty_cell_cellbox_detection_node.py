#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String # 발행할 메시지 타입

class InverseGrayDetectionNode:
    def __init__(self):
        rospy.init_node('gray_detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.detection_pub = rospy.Publisher('/inverse_gray_detection_result', String, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        # === 그리드 설정 ===
        self.P1, self.P2, self.P3 = (105,77), (547,73), (105,433)
        self.cols, self.rows = 4, 3
        self.active_cells = {(1,1),(1,3),(2,2),(2,4),(3,1),(3,3)}
        self.gray_history = [] # 안정화용

        # === 🚨 [수정] 검정색 픽셀 감지 임계값 (HSV 사용) ===
        # 검정색: V(명도)가 낮음 (V <= 60)
        self.lower_black, self.upper_black = np.array([0, 0, 0]), np.array([150, 150, 150])
        
        # 🚨 [수정] 검정색 픽셀의 최소 면적 비율 (물체가 있다고 판단하는 기준)
        self.BLACK_PIXEL_RATIO = 0.77     # 감지 최소 면적 비율 (90% 이상일 때 DETECTED)

        # 📌 우선순위 리스트 (문자열 정렬 순서로 가장 높은 우선순위 결정)
        self.priority_list = sorted([f"{r}-{c}" for r, c in self.active_cells])

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            result_frame = self.black_area_detect(frame) # 함수 이름 'black_area_detect'로 변경
            cv2.imshow("Black Area Grid Cell Detection", result_frame)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Detection Error: {e}")

    def black_area_detect(self, frame):
        display_frame = frame.copy() 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # HSV 변환
        
        xs = np.linspace(self.P1[0], self.P2[0], self.cols+1)
        ys = np.linspace(self.P1[1], self.P3[1], self.rows+1)
        detected_black = set() # 감지된 '검정색 물체' 셀 목록

        for i in range(self.rows):
            for j in range(self.cols):
                r, c = i+1, j+1
                x1, x2 = int(xs[j]), int(xs[j+1])
                y1, y2 = int(ys[i]), int(ys[i+1])

                # 영역 유효성 검사
                if y2 <= y1 or x2 <= x1:
                    continue
                
                cell_roi = hsv[y1:y2, x1:x2]
                cell_area = cell_roi.shape[0] * cell_roi.shape[1]

                # 1. HSV 임계값을 사용하여 검정색 픽셀 마스크 생성
                black_mask = cv2.inRange(cell_roi, self.lower_black, self.upper_black)
                
                # 2. 검정색 픽셀의 개수 및 비율 계산
                black_pixel_count = np.sum(black_mask == 255)
                current_ratio = black_pixel_count / cell_area if cell_area > 0 else 0
                
                cell_name = f"{r}-{c}"
                color = (80,80,80) # 기본 회색 (비활성)
                text = f"{cell_name} ({current_ratio:.2f})"
                
                # 📌 시각화는 원본 BGR 프레임 복사본(display_frame)에 수행
                if (r,c) in self.active_cells:
                    
                    # 🚨 [감지 조건] 검정색 픽셀 비율이 BLACK_PIXEL_RATIO 이상일 때 (DETECTED)
                    if current_ratio >= self.BLACK_PIXEL_RATIO:
                        detected_black.add(cell_name)
                        color = (255,165,0) # 주황색 (물체 감지)
                        text = f"DETECTED ({current_ratio:.2f})"
                    else:
                        # 🚨 [미감지] 검정색 물체가 없다고 판단 (NOT DETECTED / EMPTY)
                        color = (0, 255, 0) # 녹색 (물체 없음)
                        text = f"EMPTY ({current_ratio:.2f})"
                        
                    # 📌 활성 셀 테두리 및 텍스트 표시
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                    cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                    cv2.putText(display_frame, text, (cx - 60, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                else:
                    # 📌 비활성 셀 테두리 표시
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 1)


        # 📌 5프레임 안정화 및 우선순위가 가장 높은 셀 하나만 발행하는 로직
        # 감지된 '검정색 물체' 셀들의 목록을 우선순위에 따라 정렬
        detected_list = sorted(list(detected_black), key=lambda x: self.priority_list.index(x) if x in self.priority_list else len(self.priority_list))
        
        is_published = False
        
        # '검정색 물체'가 감지된 경우
        if detected_list:
            highest_priority_cell = detected_list[0]
            # 히스토리에 (최우선 순위 셀) 문자열을 포함하는 frozenset을 저장
            self.gray_history.append(frozenset({highest_priority_cell}))
        else:
            self.gray_history.append(frozenset()) # 감지된 셀이 없으면 빈 세트 추가

        if len(self.gray_history) > 5: self.gray_history.pop(0)
        
        # 5프레임 동안 감지된 셀이 존재하고, 그 셀이 모두 동일한 최우선 순위 셀일 경우
        if (len(self.gray_history) == 5 and 
            all(h == self.gray_history[0] for h in self.gray_history) and 
            len(self.gray_history[0]) == 1):
            
            # 최우선 순위 셀의 문자열을 추출하여 발행
            result_str = list(self.gray_history[0])[0] 
            self.detection_pub.publish(String(result_str))
            rospy.loginfo(f"✨ Black Object Stable Detection Published (Highest Priority): {result_str}")
            self.gray_history.clear() 
            is_published = True
        
        # 🚨 [추가] 안정화된 감지에 실패했을 경우 터미널에 로그 출력
        if not is_published:
            rospy.logwarn(f"🚧 Black Object NOT DETECTED (History len: {len(self.gray_history)}).")
        
        return display_frame

if __name__ == '__main__':
    try:
        node = InverseGrayDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()