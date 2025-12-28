#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class DarkGridDetectionNode:
    def __init__(self):
        rospy.init_node('dark_grid_detection_node', anonymous=True)
        self.bridge = CvBridge()
        
        self.detection_pub = rospy.Publisher('/darkgrid_detection_result', String, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # 📌📌 왜곡 보정 파라미터 (Camera Calibration 결과) 📌📌
        # !!! 경고: 이 값들은 예시입니다. 실제 카메라 캘리브레이션 값으로 반드시 변경해야 합니다. !!!
        # Camera Matrix (K)
        self.camera_matrix = np.array([
            [600.0, 0.0, 640.0],    # fx, 0, cx
            [0.0, 600.0, 360.0],    # 0, fy, cy
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Distortion Coefficients (D): k1, k2, p1, p2, k3...
        self.dist_coeffs = np.array([
            0.1, -0.05, 0.001, 0.001, 0.0 
        ], dtype=np.float64)
        
        # === 감지 파라미터 ===
        self.dark_intensity_thresh = 90   
        # 🚨 경고: 이 좌표들은 왜곡 보정된 이미지에 맞춰 다시 측정하고 변경해야 합니다. 🚨
        self.P1, self.P2, self.P3 = (105,77), (547,73), (105,433)
        self.cols, self.rows = 4, 3
        # 로봇이 Place 가능한 셀만 활성화 셀로 정의하는 것이 좋습니다.
        self.active_cells = {(1,1),(1,3),(2,2),(2,4),(3,1),(3,3)} 
        self.gray_history = []
        rospy.loginfo("--- 🖤 Dark Grid Detection Node 초기화 완료. ---")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 1. 🚨 왜곡 보정 (Undistortion) 🚨
            processed_frame = frame
            
            # 파라미터가 유효한지 확인하고 왜곡 보정 수행
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                h, w = frame.shape[:2]
                
                # 최적의 새 카메라 행렬 계산
                new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                    self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
                )
                
                # 왜곡 보정 적용
                undistorted_frame = cv2.undistort(frame, self.camera_matrix, 
                                                  self.dist_coeffs, None, new_camera_matrix)
                
                # 왜곡 보정 후 ROI(관심 영역) 자르기
                x, y, w_roi, h_roi = roi
                if w_roi > 0 and h_roi > 0:
                    processed_frame = undistorted_frame[y:y+h_roi, x:x+w_roi]
                else:
                    processed_frame = undistorted_frame
            
            # 2. Dark Grid 감지 실행 (보정된 이미지 사용)
            result_frame = self.dark_detect(processed_frame)
            cv2.imshow("Dark Grid Cell Detection", result_frame)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Dark Grid Detection Error: {e}")
            
    def dark_detect(self, frame):
        # 왜곡 보정 후 이미지의 크기가 원본과 다를 수 있으므로 그레이 변환
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # --- 그리드 좌표 계산 로직 ---
        xs = np.linspace(self.P1[0], self.P2[0], self.cols+1)
        ys = np.linspace(self.P1[1], self.P3[1], self.rows+1)
        detected = set()

        for i in range(self.rows):
            for j in range(self.cols):
                r, c = i+1, j+1
                x1, x2 = int(xs[j]), int(xs[j+1])
                y1, y2 = int(ys[i]), int(ys[i+1])
                
                # 범위 체크
                if x1 < x2 and y1 < y2 and x1 >= 0 and y1 >= 0 and x2 <= frame.shape[1] and y2 <= frame.shape[0]:
                    cell_region = gray[y1:y2, x1:x2]
                    
                    if cell_region.size > 0:
                        # 📌 핵심: 평균 밝기 계산
                        mean_intensity = np.mean(cell_region)
                    else:
                        mean_intensity = 255 # 영역이 비었으면 밝은 것으로 간주
                    
                    if (r,c) in self.active_cells:
                        color = (0,255,0)
                        text = f"{mean_intensity:.0f}"
                        
                        # 📌 감지 로직: 평균 밝기가 임계값 이하일 때 (Dark Grid)
                        if mean_intensity < self.dark_intensity_thresh:
                            detected.add(f"{r}-{c}")
                            color = (0,0,255)
                            text = f"Detected ({mean_intensity:.0f})"
                        
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                        cv2.putText(frame, f"{r}-{c} {text}", (cx-60, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                    else:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (80,80,80), 1)

        # --- 안정화 로직 ---
        self.gray_history.append(frozenset(detected))
        if len(self.gray_history) > 3:
            self.gray_history.pop(0)

        if (len(self.gray_history) == 3 and
            all(h == self.gray_history[0] for h in self.gray_history) and
            len(detected) > 0):
            
            # 🚨🚨🚨 수정된 핵심 로직: 감지된 셀 중 첫 번째 셀만 선택하여 발행 🚨🚨🚨
            stable_detected_set = self.gray_history[0]
            
            if stable_detected_set:
                # 감지된 유효한 셀들(예: {"1-1", "2-2"})을 사전순으로 정렬하여 첫 번째 셀만 선택
                # 이렇게 하면 컨트롤러가 예상하는 단일 키 형식(예: "1-1")이 발행됩니다.
                result_str = sorted(list(stable_detected_set))[0] 
                
                self.detection_pub.publish(String(result_str))
                rospy.loginfo(f"🖤 Dark Grid Stable Detection Published: {result_str}")
                self.gray_history.clear()
            # 🚨🚨🚨 수정 끝 🚨🚨🚨

        return frame

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DarkGridDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()