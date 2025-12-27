# coding=utf-8
import sys
import cv2
import numpy as np
from ultralytics import YOLO
import PySpin
import socket
import math
import struct

# PyQt5 Imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                             QVBoxLayout, QHBoxLayout, QGridLayout, QTextEdit,
                             QGroupBox, QScrollArea, QPushButton)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import (QThread, pyqtSignal, pyqtSlot, Qt,
                          QMutex, QWaitCondition)

HOST = '192.168.0.18' # 통신을 위해 serial 번호 바꾸기
PORT = 9999 # 받는 쪽과 포트 번호 맞추기

# 변수 Threshold 값
EDGE_RING_THICKNESS = 15    # upper 카메라 ring 두께 픽셀
CANNY_LOW, CANNY_HIGH = 210, 250    # canny edge threshold
OUTER_MIN_RADIUS = 280 # 바깥 반지름 최소값
OUTER_MAX_RADIUS = 350 # 바깥 반지름 최대값
INNER_MIN_RADIUS = 180 # 안쪽 반지름 최소값
INNER_MAX_RADIUS = 250 # 안쪽 반지름 최대값
CENTER_OFFSET_TOLERANCE = 70 # 원의 가운데가 이미지 상에 가운데 지나갈때 state 판단. 가운데의 offset값
MIN_PIXEL_AREA_THRESHOLD = 95000    #원 안에 픽셀 threshold값
CELL_2_TIMEOUT_FRAMES = 100  # 2번 셀을 기다릴 최대 프레임 수

# 통신을 원하면 주석처리 되어있는 통신 코드 활성화
def create_connection(host=HOST, port=PORT, timeout=10):
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        print("통신 연결 성공")
        return s
    except Exception as e:
        print(f"통신 연결 실패: {e}")
        return None

# flag 통신
def send_flag(sock, flag):
    try:
        flag_int = int(flag)
        packed_data = struct.pack('B', flag_int)

        sock.sendall(packed_data)
        print(f"Flag '{flag}' (Bytes: {packed_data}) 전송 완료")
    except ValueError:
        print(f"전송 오류: '{flag}'를 정수로 변환할 수 없습니다.")
    except Exception as e:
        print(f"전송 오류: {e}")


class ProcessingThread(QThread):
    change_pixmap_cell = pyqtSignal(np.ndarray)
    change_pixmap_surface = pyqtSignal(np.ndarray)
    # change_pixmap_edge 및 change_pixmap_static 시그널 제거됨
    change_pixmap_cell1_capture = pyqtSignal(np.ndarray)  # Cell 1 캡처 이미지 시그널 추가
    change_pixmap_cell2_capture = pyqtSignal(np.ndarray)  # Cell 2 캡처 이미지 시그널 추가
    log_message = pyqtSignal(str)
    update_status_signal = pyqtSignal(int, str, str, str)

    def __init__(self):
        super().__init__()
        self._running = True
        self._paused = False
        self.mutex = QMutex()
        self.pause_cond = QWaitCondition()
        self.cell_1_result = None

    def stop(self):
        self.mutex.lock()
        self._running = False
        self.mutex.unlock()
        self.pause_cond.wakeAll()

    def pause(self):
        self.mutex.lock()
        self._paused = True
        self.mutex.unlock()

    def resume(self):
        self.mutex.lock()
        self._paused = False
        self.mutex.unlock()
        self.pause_cond.wakeAll()

    def run(self):
        system = None
        cam_list = None
        cam1 = None
        cam2 = None
        sock = None

        try:
            system = PySpin.System.GetInstance()
            version = system.GetLibraryVersion()
            self.log_message.emit(
                'Library version: %d.%d.%d.%d' % (version.major, version.minor, version.type, version.build))

            cam_list = system.GetCameras()
            num_cams = cam_list.GetSize()
            if num_cams < 2:
                self.log_message.emit(f'FLIR 카메라 2대가 필요합니다. 현재 감지된 카메라 수: {num_cams}')
                cam_list.Clear()
                system.ReleaseInstance()
                return

            # FLIR 카메라 2대 사용
            # 서로 반전 될수 있음 -> 0, 1 인덱스 바꿔주기
            cam1 = cam_list[0]  # Cell / Edge / Thickness
            cam2 = cam_list[1]  # Surface YOLO
            self.log_message.emit("Spinnaker 카메라 2대 연결 완료")

            cam1.Init()
            cam2.Init()

            # 버퍼 처리 모드 및 Acquisition Mode 설정 함수들
            def set_newestonly(cam):
                sNodemap = cam.GetTLStreamNodeMap()
                node_bufferhandling_mode = PySpin.CEnumerationPtr(sNodemap.GetNode('StreamBufferHandlingMode'))
                if PySpin.IsReadable(node_bufferhandling_mode) and PySpin.IsWritable(node_bufferhandling_mode):
                    node_bufferhandling_mode.SetIntValue(
                        node_bufferhandling_mode.GetEntryByName('NewestOnly').GetValue())

            def set_continuous(cam):
                nodemap = cam.GetNodeMap()
                node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode('AcquisitionMode'))
                if PySpin.IsReadable(node_acquisition_mode) and PySpin.IsWritable(node_acquisition_mode):
                    node_acquisition_mode.SetIntValue(node_acquisition_mode.GetEntryByName('Continuous').GetValue())
                    self.log_message.emit('Acquisition mode set to continuous...')

            # 설정 적용
            set_newestonly(cam1)
            set_newestonly(cam2)
            set_continuous(cam1)
            set_continuous(cam2)

            sock = create_connection()
            if not sock:
                self.log_message.emit("통신 연결 실패. 스레드 종료.")
                cam1.DeInit()
                cam2.DeInit()
                cam_list.Clear()
                system.ReleaseInstance()
                return

            # YOLO 모델 로드 (Surface 용)
            surface_model_path = '../../train/runs/yolov8s_surface_defect_v1/weights/best.pt'
            try:
                surface_model = YOLO(surface_model_path)
                self.log_message.emit(f"Surface model loaded: {surface_model_path}")
            except Exception as e:
                self.log_message.emit(f"Error loading YOLO: {e}")
                return

            processor = PySpin.ImageProcessor()
            processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)

            cam1.BeginAcquisition()
            cam2.BeginAcquisition()
            self.log_message.emit("카메라 1, 2 Acquisition 시작됨")

            is_object_centered_previously = False
            current_cell_index = 1
            cell_2_timeout_counter = 0  # 2번 셀 타임아웃 카운터
            self.update_status_signal.emit(1, "WAITING", "---", "")
            self.update_status_signal.emit(2, "---", "---", "")

            while self._running:
                # 일시정지 처리
                self.mutex.lock()
                while self._paused and self._running:
                    self.pause_cond.wait(self.mutex)
                self.mutex.unlock()

                if not self._running:
                    break

                cell_2_detected_this_frame = False  # 프레임마다 리셋

                try:
                    # cam1: Cell / Edge 처리
                    image_result1 = cam1.GetNextImage(1000)
                    if image_result1.IsIncomplete():
                        image_result1.Release()
                        continue

                    image_converted1 = processor.Convert(image_result1, PySpin.PixelFormat_BGR8)
                    color_image_data = image_converted1.GetNDArray().copy()
                    h, w, _ = color_image_data.shape
                    roi = color_image_data[h // 2 - 500: h // 2 + 500, w // 2 - 500: w // 2 + 500]
                    gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

                    h_roi, w_roi = gray_image.shape
                    roi_center_x = w_roi // 2
                    blurred_image = cv2.GaussianBlur(gray_image, (11, 11), 2)
                    annotated_frame = roi.copy()

                    msg = "OK"
                    failure_reason = ""
                    should_run_surface_detection = False

                    # 바깥 원 검출
                    outer_circles = cv2.HoughCircles(
                        image=blurred_image, method=cv2.HOUGH_GRADIENT, dp=1,
                        minDist=gray_image.shape[0] // 4, param1=200, param2=30,
                        minRadius=OUTER_MIN_RADIUS, maxRadius=OUTER_MAX_RADIUS
                    )

                    if outer_circles is not None:
                        circles = np.uint16(np.around(outer_circles))
                        largest_circle = max(circles[0, :], key=lambda c: c[2])
                        cx, cy, r_outer = largest_circle
                        center = (cx, cy)
                        circle_area = np.pi * (r_outer ** 2)

                        if circle_area >= MIN_PIXEL_AREA_THRESHOLD:
                            should_run_surface_detection = True

                            # 균열 검출 로직 (Edge Ring)
                            mask_outer = np.zeros_like(gray_image)
                            cv2.circle(mask_outer, center, r_outer + EDGE_RING_THICKNESS, 255, -1)
                            cv2.circle(mask_outer, center, r_outer - EDGE_RING_THICKNESS, 0, -1)
                            ring_roi = cv2.bitwise_and(gray_image, mask_outer)
                            edges = cv2.Canny(ring_roi, CANNY_LOW, CANNY_HIGH)

                            # 윤곽선 추출 및 crack_reason 판단 로직 유지
                            contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL,
                                                           cv2.CHAIN_APPROX_SIMPLE)
                            MIN_CONTOUR_AREA = 500
                            large_contours = [cnt for cnt in contours if
                                              cv2.contourArea(cnt) > MIN_CONTOUR_AREA]

                            CIRCULARITY_THRESHOLD = 0.87

                            is_crack_detected = False
                            crack_reason = ""

                            if len(large_contours) == 0:
                                is_crack_detected = True
                                crack_reason = "No edge ring"
                            else:
                                cnt = large_contours[0]
                                perimeter = cv2.arcLength(cnt, True)
                                area = cv2.contourArea(cnt)
                                circularity = 0
                                if perimeter > 0:
                                    circularity = (4 * math.pi * area) / (perimeter ** 2)
                                if circularity < CIRCULARITY_THRESHOLD:
                                    is_crack_detected = True
                                    crack_reason = f"Crack detected (Circularity: {circularity:.2f})"
                                else:
                                    is_crack_detected = False

                            # 객체가 중앙에 위치했는지 확인
                            if abs(int(cx) - roi_center_x) < CENTER_OFFSET_TOLERANCE:
                                if is_crack_detected:
                                    print(f"Crack detected: {crack_reason}")
                                    result_text = f"NG - {crack_reason}"
                                    color = (0, 0, 255)
                                    msg = "NOK"
                                    failure_reason = crack_reason

                                    if len(large_contours) > 0:
                                        cv2.drawContours(annotated_frame, large_contours, -1, color, 2)
                                else:
                                    print("Normal Edge (Circularity OK)")
                                    result_text = "OK - No crack"
                                    color = (0, 255, 0)

                                # 판정 결과를 바탕으로 원본 이미지에 원 그리기
                                cv2.circle(annotated_frame, center, r_outer, color, 3)
                                cv2.circle(annotated_frame, center, 2, (255, 255, 255), 3)

                            # 두께 측정 로직 유지
                            inner_circles = cv2.HoughCircles(
                                image=blurred_image, method=cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                param1=200, param2=30, minRadius=INNER_MIN_RADIUS, maxRadius=INNER_MAX_RADIUS
                            )
                            if inner_circles is not None:
                                circles_inner = np.uint16(np.around(inner_circles))
                                largest_inner_circle = max(circles_inner[0, :], key=lambda c: c[2])
                                cx_inner, cy_inner, r_inner = largest_inner_circle
                                center_inner = (cx_inner, cy_inner)
                                cv2.circle(annotated_frame, center_inner, r_inner, (0, 255, 255), 3)
                                thickness = r_outer - r_inner
                            else:
                                image_result1.Release()
                                continue
                        else:
                            msg = "NOK"
                            failure_reason = "Circle too small"
                            should_run_surface_detection = False
                    else:
                        msg = "NOK"
                        failure_reason = "No circle found"
                        should_run_surface_detection = False

                    self.change_pixmap_cell.emit(annotated_frame)
                    image_result1.Release()

                    # cam2: Surface (YOLO) 처리
                    image_result2 = cam2.GetNextImage(1000)
                    if image_result2.IsIncomplete():
                        image_result2.Release()
                        continue

                    image_converted2 = processor.Convert(image_result2, PySpin.PixelFormat_BGR8)
                    frame2 = image_converted2.GetNDArray().copy()

                    h_usb, w_usb, _ = frame2.shape

                    frame_roi = frame2[h_usb // 2 - 948:h_usb // 2 + 950,
                    w_usb // 2 - 1350:w_usb // 2 + 1350]

                    surface_display_frame = frame_roi.copy()

                    if should_run_surface_detection:
                        surface_results = surface_model(frame_roi, verbose=False)
                        surface_display_frame = surface_results[0].plot()
                        num_faults = len(surface_results[0])

                        if num_faults > 1:
                            if msg == "OK":
                                failure_reason = f"Surface faults >= {num_faults}"
                            else:
                                failure_reason += f" & Surface faults >= {num_faults}"
                            msg = "NOK"

                        is_centered_x = abs(int(cx) - roi_center_x) < CENTER_OFFSET_TOLERANCE

                        # Cell1/Cell2 판정 로직
                        if is_centered_x and not is_object_centered_previously:
                            is_object_centered_previously = True
                            numeric_msg = ""
                            result_msg = msg
                            reason_msg = failure_reason

                            # 캡처된 프레임 복사
                            captured_frame = annotated_frame.copy()

                            if current_cell_index == 1:
                                # 첫 번째 셀 결과 저장
                                self.cell_1_result = msg  # 'OK' 또는 'NOK' 저장

                                log_str = f"--- Cell 1 DETECTED --- Result: {msg}. Waiting for Cell 2..."
                                self.log_message.emit(log_str)

                                self.change_pixmap_cell1_capture.emit(captured_frame)

                                self.update_status_signal.emit(1, "DETECTED", result_msg, reason_msg)
                                self.update_status_signal.emit(2, "WAITING", "---", "")
                                current_cell_index = 2
                                cell_2_timeout_counter = 0  # 2번 셀 타이머 시작/리셋

                            elif current_cell_index == 2:
                                # 두 번째 셀 결과와 조합
                                if self.cell_1_result == "OK" and msg == "OK":
                                    numeric_msg = "1"  # 둘다 OK
                                elif self.cell_1_result == "NOK" and msg == "OK":
                                    numeric_msg = "2"  # 첫번째 NOK, 두번째 OK
                                elif self.cell_1_result == "OK" and msg == "NOK":
                                    numeric_msg = "3"  # 첫번째 OK, 두번째 NOK
                                elif self.cell_1_result == "NOK" and msg == "NOK":
                                    numeric_msg = "4"  # 둘다 NOK

                                log_str = (f"--- Cell 2 DETECTED & COMBINED --- "
                                           f"Cell 1:{self.cell_1_result}, Cell 2:{msg}. Sending Flag: {numeric_msg}")
                                self.log_message.emit(log_str)
                                send_flag(sock, numeric_msg)  # 통신 활성화

                                self.change_pixmap_cell2_capture.emit(captured_frame)

                                self.update_status_signal.emit(2, "DETECTED", result_msg, reason_msg)
                                self.update_status_signal.emit(1, "WAITING", "__NO_CHANGE__", "__NO_CHANGE__")
                                current_cell_index = 1
                                cell_2_timeout_counter = 0
                                cell_2_detected_this_frame = True
                                self.cell_1_result = None  # Cell 1 결과 초기화
                    else:
                        if is_object_centered_previously:
                            self.log_message.emit("Object detection failed (no circle). Resetting center flag.")
                            is_object_centered_previously = False

                    # 2번 셀을 기다리는 중이고, 이번 프레임에서 2번 셀이 감지되지 않았다면
                    if current_cell_index == 2 and not cell_2_detected_this_frame:
                        cell_2_timeout_counter += 1  # 카운터 증가

                        # 카운터가 설정된 프레임 수를 초과하면 (타임아웃)
                        if cell_2_timeout_counter > CELL_2_TIMEOUT_FRAMES:
                            if self.cell_1_result == "OK":
                                numeric_msg = "5"  # 첫번째 OK, 두번째 없음
                            elif self.cell_1_result == "NOK":
                                numeric_msg = "6"  # 첫번째 NOK, 두번째 없음
                            else:
                                numeric_msg = "5"

                            log_str = f"--- Cell 2 NOT DETECTED (Timeout) --- Sending Flag: {numeric_msg}"
                            self.log_message.emit(log_str)
                            send_flag(sock, numeric_msg)

                            # GUI 업데이트
                            self.update_status_signal.emit(2, "UNDETECTED", "N/A (Timeout)", "Cell 2 not found")

                            # 다시 1번 셀을 기다리는 상태로 리셋
                            current_cell_index = 1
                            cell_2_timeout_counter = 0
                            self.update_status_signal.emit(1, "WAITING", "---", "")
                            self.cell_1_result = None

                    self.change_pixmap_surface.emit(surface_display_frame)
                    image_result2.Release()

                except PySpin.SpinnakerException as ex:
                    self.log_message.emit(f"Spinnaker exception in loop: {ex}")
                    continue
                except Exception as e:
                    self.log_message.emit(f"General exception in loop: {e}")
                    continue

        except PySpin.SpinnakerException as ex:
            self.log_message.emit(f'Error during setup: {ex}')
        except Exception as e:
            self.log_message.emit(f'Error during setup: {e}')
        finally:
            self.log_message.emit("Cleaning up resources...")

            # Acquisition 종료 및 카메라 DeInit
            for cam in (cam1, cam2):
                try:
                    if cam and cam.IsStreaming():
                        cam.EndAcquisition()
                except Exception:
                    pass
                try:
                    if cam and cam.IsInitialized():
                        cam.DeInit()
                except Exception:
                    pass

            # 소켓 연결 해제
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

            if cam_list:
                cam_list.Clear()
            if system:
                system.ReleaseInstance()
            self.log_message.emit("Cleanup complete. Thread finished.")


class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Machine Vision Inspection")
        self.setGeometry(100, 100, 1600, 900)

        self.is_paused = False

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)

        image_grid_layout = QGridLayout()

        self.label_cell = self.create_display_label()
        image_grid_layout.addWidget(self.create_title_label("Cell Detection (Circle)"), 0, 0)
        image_grid_layout.addWidget(self.label_cell, 1, 0)

        self.label_surface = self.create_display_label()
        image_grid_layout.addWidget(self.create_title_label("Surface Detecting"), 0, 1)
        image_grid_layout.addWidget(self.label_surface, 1, 1)

        # 하단 이미지 디스플레이: Cell 1, Cell 2 캡처 이미지
        self.label_cell1_capture = self.create_display_label()
        image_grid_layout.addWidget(self.create_title_label("Cell 1 Inspection Result"), 2, 0)
        image_grid_layout.addWidget(self.label_cell1_capture, 3, 0)

        self.label_cell2_capture = self.create_display_label()
        image_grid_layout.addWidget(self.create_title_label("Cell 2 Inspection Result"), 2, 1)
        image_grid_layout.addWidget(self.label_cell2_capture, 3, 1)

        main_layout.addLayout(image_grid_layout, 3)

        # 오른쪽: 로그 및 제어 버튼
        right_pane_layout = QVBoxLayout()

        right_pane_layout.addWidget(self.create_title_label("Logs"))
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(200)
        right_pane_layout.addWidget(self.log_box)

        # Cell 상태 섹션
        cell_status_group = QGroupBox("Cell Inspection Status")
        cell_status_group.setStyleSheet("QGroupBox { font-size: 16px; font-weight: bold; margin-top: 10px; }")

        cell_status_internal_layout = QVBoxLayout()
        cell_status_internal_layout.addWidget(self.create_status_group("Cell 1"))
        cell_status_internal_layout.addWidget(self.create_status_group("Cell 2"))
        cell_status_internal_layout.addStretch()
        cell_status_group.setLayout(cell_status_internal_layout)

        right_pane_layout.addWidget(cell_status_group)

        # 제어 버튼 섹션
        self.start_button = QPushButton("Start")
        self.start_button.setStyleSheet("background-color: #4CAF50; color: white; height: 30px; font-weight: bold;")

        self.pause_button = QPushButton("Pause")
        self.pause_button.setStyleSheet(
            "background-color: #FFC107; color: black; height: 30px; font-weight: bold;")

        self.stop_button = QPushButton("Stop")
        self.stop_button.setStyleSheet("background-color: #F44336; color: white; height: 30px; font-weight: bold;")

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.pause_button)
        button_layout.addWidget(self.stop_button)

        right_pane_layout.addLayout(button_layout)
        right_pane_layout.addStretch()

        main_layout.addLayout(right_pane_layout, 1)

        # 스레드 생성
        self.thread = ProcessingThread()
        self.connect_signals()

        # 버튼 연결 및 초기 상태 설정
        self.start_button.clicked.connect(self.start_processing)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.stop_button.clicked.connect(self.stop_processing)

        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)

    def create_status_group(self, title):
        group_box = QGroupBox(title)
        group_box.setStyleSheet("QGroupBox { font-size: 16px; font-weight: bold; margin-top: 10px; }")
        v_layout = QVBoxLayout()

        # State
        state_label_title = QLabel("State:")
        state_label_title.setStyleSheet("font-size: 14px; font-weight: normal;")
        state_label_value = QLabel("---")

        # Result
        result_label_title = QLabel("Result:")
        result_label_title.setStyleSheet("font-size: 14px; font-weight: normal; margin-top: 5px;")
        result_label_value = QLabel("---")

        # Reason
        reason_label_title = QLabel("Reason:")
        reason_label_title.setStyleSheet("font-size: 14px; font-weight: normal; margin-top: 5px;")
        reason_label_value = QLabel("---")

        v_layout.addWidget(state_label_title)
        v_layout.addWidget(state_label_value)
        v_layout.addWidget(result_label_title)
        v_layout.addWidget(result_label_value)
        v_layout.addWidget(reason_label_title)
        v_layout.addWidget(reason_label_value)
        v_layout.addStretch()

        group_box.setLayout(v_layout)

        if title == "Cell 1":
            self.label_cell1_status = state_label_value
            self.label_cell1_result = result_label_value
            self.label_cell1_reason_title = reason_label_title
            self.label_cell1_reason_value = reason_label_value
        else:
            self.label_cell2_status = state_label_value
            self.label_cell2_result = result_label_value
            self.label_cell2_reason_title = reason_label_title
            self.label_cell2_reason_value = reason_label_value

        reason_label_title.hide()
        reason_label_value.hide()

        return group_box

    def create_display_label(self):
        label = QLabel()
        label.setMinimumSize(480, 360)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("border: 1px solid #555; background-color: #333;")
        return label

    def create_title_label(self, text):
        label = QLabel(text)
        label.setStyleSheet("font-size: 16px; font-weight: bold; margin-top: 10px;")
        label.setAlignment(Qt.AlignCenter)
        return label

    def connect_signals(self):
        self.thread.change_pixmap_cell.connect(lambda img: self.update_image(img, self.label_cell))
        self.thread.change_pixmap_surface.connect(lambda img: self.update_image(img, self.label_surface))

        self.thread.change_pixmap_cell1_capture.connect(lambda img: self.update_image(img, self.label_cell1_capture))
        self.thread.change_pixmap_cell2_capture.connect(lambda img: self.update_image(img, self.label_cell2_capture))

        self.thread.log_message.connect(self.update_log)
        self.thread.update_status_signal.connect(self.update_status_display)

        self.thread.finished.connect(self.on_thread_finished)

    def closeEvent(self, event):
        self.update_log("Main window closing. Stopping thread...")
        if self.thread.isRunning():
            self.thread.stop()
            self.thread.wait()
        event.accept()

    @pyqtSlot(str)
    def update_log(self, message):
        self.log_box.append(message)
        self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())

    @pyqtSlot(int, str, str, str)
    def update_status_display(self, cell_num, status, result, reason):
        status_label = None
        result_label = None
        reason_title_label = None
        reason_value_label = None

        if cell_num == 1:
            status_label = self.label_cell1_status
            result_label = self.label_cell1_result
            reason_title_label = self.label_cell1_reason_title
            reason_value_label = self.label_cell1_reason_value
        elif cell_num == 2:
            status_label = self.label_cell2_status
            result_label = self.label_cell2_result
            reason_title_label = self.label_cell2_reason_title
            reason_value_label = self.label_cell2_reason_value
        else:
            return

        base_style = "font-size: 16px; padding: 5px; border-radius: 5px;"

        if status == "DETECTED":
            status_color_style = "background-color: #4CAF50; color: white;"
        elif status == "WAITING":
            status_color_style = "background-color: #FFC107; color: black;"
        else:
            status_color_style = "background-color: #607D8B; color: white;"
        status_label.setStyleSheet(base_style + status_color_style)
        status_label.setText(status)

        if result != "__NO_CHANGE__":

            if "NOK" in result:
                result_color_style = "background-color: #F44336; color: white;"
            elif "OK" in result:
                result_color_style = "background-color: #4CAF50; color: white;"
            elif "N/A" in result:
                result_color_style = "background-color: #9E9E9E; color: white;"
            else:
                result_color_style = "background-color: #607D8B; color: white;"
            result_label.setStyleSheet(base_style + result_color_style)
            result_label.setText(result)

            if result == "NOK" or "N/A" in result:
                reason_value_label.setText(reason)
                reason_value_label.setStyleSheet(
                    base_style + "background-color: #333; color: #F44336; font-weight: bold;")
                reason_title_label.show()
                reason_value_label.show()
            else:
                reason_value_label.setText("---")
                reason_title_label.hide()
                reason_value_label.hide()

    @pyqtSlot(np.ndarray, QLabel, bool)
    def update_image(self, cv_img, label, is_gray=False):
        try:
            if is_gray:
                h, w = cv_img.shape
                bytes_per_line = w
                qt_img_format = QImage.Format_Grayscale8
                qt_img = QImage(cv_img.data, w, h, bytes_per_line, qt_img_format)
            else:
                rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_img_format = QImage.Format_RGB888
                qt_img = QImage(rgb_image.data, w, h, bytes_per_line, qt_img_format)

            pixmap = QPixmap.fromImage(qt_img)
            scaled_pixmap = pixmap.scaled(label.width(), label.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            label.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"Error updating image: {e}")

    @pyqtSlot()
    def start_processing(self):
        if self.thread.isFinished():
            self.update_log("Creating new processing thread...")
            self.thread = ProcessingThread()
            self.connect_signals()

        if not self.thread.isRunning():
            self.update_log("Starting processing thread...")
            self.thread.start()
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.pause_button.setEnabled(True)
            self.is_paused = False
            self.pause_button.setText("Pause")
        else:
            self.update_log("Thread is already running.")

    @pyqtSlot()
    def toggle_pause(self):
        if not self.thread.isRunning():
            return

        self.is_paused = not self.is_paused

        if self.is_paused:
            self.thread.pause()
            self.update_log("Processing paused.")
            self.pause_button.setText("Resume")
        else:
            self.thread.resume()
            self.update_log("Processing resumed.")
            self.pause_button.setText("Pause")

    @pyqtSlot()
    def stop_processing(self):
        if self.thread.isRunning():
            self.update_log("Signaling thread to stop...")
            self.thread.stop()
            self.stop_button.setEnabled(False)
            self.pause_button.setEnabled(False)
        else:
            self.update_log("Thread is not running.")

    @pyqtSlot()
    def on_thread_finished(self):
        self.update_log("Processing thread has finished.")
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self.is_paused = False
        self.pause_button.setText("Pause")


if __name__ == '__main__':
    app = QApplication(sys.argv)

    app.setStyle('Fusion')
    try:
        from PyQt5.QtGui import QPalette
        from PyQt5.QtGui import QColor

        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, Qt.black)
        app.setPalette(palette)
    except ImportError:
        pass

    ex = App()
    ex.show()
    sys.exit(app.exec_())