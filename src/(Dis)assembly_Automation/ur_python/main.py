#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
import rospy
import math
import numpy as np
from math import pi
import time as pytime
from time import time
import serial

from move_group_python_interface import MoveGroupPythonInterface
from geometry_msgs.msg import Point
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion


# ------------------------------------------------------------
# 간단 토픽 래치
# ------------------------------------------------------------
class TopicLatch:
    def __init__(self, ttl=3.0):
        self.ttl = ttl
        self.data = None
        self.stamp = 0.0
    def update(self, msg):
        self.data = msg
        self.stamp = time()
    def alive(self):
        return (time() - self.stamp) <= self.ttl
    def get(self, default=None):
        return self.data if self.alive() else default


class AsmFsmNode:
    def __init__(self):
        # rospy.init_node("asm_fsm_node", anonymous=True)
        self.ur5e = MoveGroupPythonInterface(real="real", gripper="gripper")

        # ✅ 초기 RPY 부호 교정 불필요
        rospy.loginfo("🧭 MoveGroupPythonInterface initialized with stabilized RPY logic (±0.1° limit + minimal rotation).")

        # ---------- 아두이노 시리얼 연결 ----------
        try:
            self.arduino = serial.Serial()
            self.arduino.port = '/dev/ttyACM0'
            self.arduino.baudrate = 9600
            self.arduino.timeout = 1

            # ★★★ reset 방지 (포트 오픈 전에 해야 함)
            self.arduino.dtr = False
            self.arduino.rts = False

            self.arduino.open()     # 이제 UNO Reset 절대 안 됨

            rospy.loginfo("🔌 Arduino Serial Connected (/dev/ttyACM) without reset")
        except Exception as e:
            rospy.logwarn(f"⚠️ Arduino Serial Connection Failed: {e}")
            self.arduino = None


        # 이하 원래 코드
        self.GRIP_HOLD = 2
        self.D2R = pi/180.0
        self.prev_orange = None
        self.ema_alpha = 0.35
        self.ema_stable_thresh_mm = 10.0  # 1mm 이하 변화 시 안정화 간주
        self.ema_min_frames = 2          # 최소 4프레임 유지 필요
        self.ema_stable_count = 0


        # ---- 토픽 래치 ----
        self.red_result        = TopicLatch(ttl=2.0)   # /red_detection/result (Point)
        self.orange_result     = TopicLatch(ttl=2.0)   # /orange_detection/result (Point)
        self.darkgrid_result   = TopicLatch(ttl=2.0)   # /darkgrid_detection_result (String)
        self.gray_result       = TopicLatch(ttl=2.0)   # /gray_detection_result (String)
        self.inv_gray_result   = TopicLatch(ttl=2.0)   # /inverse_gray_detection_result (String)
        self.inv_grid_result   = TopicLatch(ttl=2.0)   # /inverse_grid_detection_result (String)
        self.fallback_circle   = TopicLatch(ttl=2.0)   # /fallback_circle/result (Point)
        self.grid1_fallback    = TopicLatch(ttl=2.0)   # /grid1_fallback/result (Point)
        self.grid2_fallback    = TopicLatch(ttl=2.0)   # /grid2_fallback/result (Point)


        # ---- 구독 ----
        rospy.Subscriber("/red_detection/result", Point, self.cb_red)
        rospy.Subscriber("/orange_detection/result", Point, self.cb_orange)
        rospy.Subscriber("/darkgrid_detection_result", String, self.cb_darkgrid)
        rospy.Subscriber("/gray_detection_result", String, self.cb_gray)
        rospy.Subscriber("/inverse_gray_detection_result", String, self.cb_inv_gray)
        rospy.Subscriber("/inverse_grid_detection_result", String, self.cb_inv_grid)
        rospy.Subscriber("/fallback_circle/result", Point, self.cb_fallback_circle)
        rospy.Subscriber("/grid1_fallback/result", Point, self.cb_grid1_fallback)
        rospy.Subscriber("/grid2_fallback/result", Point, self.cb_grid2_fallback)


        # ---- 조인트 (deg) ----
        self.ASM_OBS_START_JOINTS          = [298.62, -102.48, 119.54, -106.94, -89.63, -61.17]
        self.RED_PLACE_POSE_1_JOINTS       = [308.07,  -92.37, 110.96, -108.60, -89.66, -51.81]
        self.ASM_OBS_GRID_JOINTS           = [243.53,  -74.39,  66.28,  -82.02, -90.00, -116.50]
        self.ASM_RED_DETECT_TRIGGER_JOINTS = [272.92, -118.99, 129.53, -100.31, -89.71,  -86.91]
        self.OBS_GRID_JOINTS               = [267.86,  -74.47,  66.36,  -81.72, -90.07,  -92.25]
        self.RED_PLACE_POSE_2_JOINTS       = [287.35, -103.92, 128.08, -108.26, -88.37,  -67.33]

        # === 새로 지정된 Grip-OFF 경유 좌표들 ===
        # A 경로 (state 1/101/201/301에서 사용)
        self.OFF_A1 = [308.44, -89.11, 116.87, -117.93, -89.93, -51.93]
        self.OFF_A2 = [308.44, -90.27, 115.27, -115.26, -89.92, -51.96]
        # B 경로 (state 3/103에서 사용)
        self.OFF_B1 = [288.55, -106.54, 133.50, -116.53, -89.89, -71.82]
        self.OFF_B2 = [288.58, -108.29, 130.91, -112.21, -89.95, -71.79]
        # state 6/106 이후 Grip OFF 위치 동일 (B2)
        self.GRIP_OFF_B2 = self.OFF_B2
        # state 5/105 이후 Grip OFF 위치 (A2)
        self.GRIP_OFF_A2 = self.OFF_A2

        # ---- 토픽별 포즈 딕셔너리 (새 좌표 반영) ----
        # /gray_detection_result → 조립용 픽 (그레이드 셀)
        self.ASM_GRID_POSES = {
            "1-1":{"start":[273.89, -84.60, 93.19, -98.45, -90.06, -86.15]},
            "1-3":{"start":[273.15, -67.90, 72.17, -94.09, -90.09, -86.80]},
            "2-2":{"start":[266.95, -78.48, 86.06, -97.27, -90.08, -93.06]},
            "2-4":{"start":[267.49, -60.76, 61.65, -90.54, -90.09, -92.57]},
            "3-1":{"start":[259.17, -88.10, 96.91, -98.58, -90.15, -100.80]},
            "3-3":{"start":[261.31, -70.86, 76.25, -95.19, -90.08, -98.61]},
        }

        self.DIS_GRID_POSES = {
            "1-1": {"start": [245.12, -87.75, 96.56, -98.68, -90.19, -114.91]},
            "1-3": {"start": [249.95, -70.67, 76.04, -95.30, -90.18, -110.01]},
            "2-2": {"start": [241.81, -77.35, 84.69, -97.10, -90.22, -118.16]},
            "2-4": {"start": [246.63, -59.80, 60.15, -90.00, -90.18, -113.43]},
            "3-1": {"start": [232.74, -82.41, 90.72, -98.17, -90.27, -127.19]},
            "3-3": {"start": [239.69, -65.02, 68.04, -92.84, -90.11, -120.19]},
        }

        # /darkgrid_detection_result → 빈 그리드 픽(조립 전)
        self.DARKGRID_POSES = {
        "1-1":{"start":[254.13,-89.30,97.65,-98.06,-90.11,-107.77],"pick":[254.10,-86.06,107.90,-111.54,-90.01,-107.82]},
        "1-3":{"start":[257.45,-71.90,77.20,-95.00,-89.84,-102.30],"pick":[257.44,-69.45,87.32,-107.60,-89.84,-102.31]},
        "2-2":{"start":[249.57,-80.41,85.10,-94.64,-89.90,-110.18],"pick":[249.40,-77.17,98.43,-111.19,-89.92,-110.35]},
        "2-4":{"start":[253.53,-62.42,60.22,-87.84,-89.81,-106.28],"pick":[253.18,-60.95,74.89,-103.90,-89.83,-106.34]},
        "3-1":{"start":[240.32,-86.43,93.50,-96.67,-89.95,-119.46],"pick":[240.31,-83.03,104.88,-111.49,-89.94,-119.43]},
        "3-3":{"start":[246.35,-69.47,70.17,-90.42,-89.79,-114.60],"pick":[245.97,-67.14,83.81,-106.32,-89.81,-113.90]},
        }
        # /inverse_gray_detection_result → fallback 상황의 회색 셀 픽
        self._FALLBACK_POSES = {
        "1-1":{"start":[282.64,-80.76,88.80,-97.86,-89.85,-77.24],"pick":[282.38,-78.32,97.93,-109.45,-89.81,-77.47]},
        "1-3":{"start":[280.40,-65.60,69.21,-94.72,-90.02,-79.52],"pick":[280.11,-63.99,78.71,-105.82,-90.02,-79.77]},
        "2-2":{"start":[275.13,-77.00,84.56,-98.48,-89.79,-84.87],"pick":[275.01,-74.55,94.24,-110.57,-89.77,-84.99]},
        "2-4":{"start":[274.12,-59.33,59.69,-91.05,-89.66,-85.73],"pick":[273.92,-58.49,70.86,-103.01,-89.54,-85.82]},
        "3-1":{"start":[268.33,-87.22,96.27,-99.81,-89.82,-91.56],"pick":[268.05,-84.59,106.33,-112.47,-89.92,-91.80]},
        "3-3":{"start":[268.64,-70.52,76.07,-96.50,-89.75,-91.21],"pick":[268.18,-68.58,86.08,-108.45,-89.77,-91.65]},
        }
        # /inverse_grid_detection_result → 분해공정 픽
        self.INVERSE_POSES = {
            "1-1":{"start":[253.74,-89.33,99.87,-100.37,-89.86,-108.12],"pick":[253.73,-84.97,110.70,-115.54,-89.85,-108.10]},
            "1-3":{"start":[257.34,-71.88,79.42,-97.27,-89.95,-102.36],"pick":[257.34,-68.50,89.85,-111.02,-89.93,-102.33]},
            "2-2":{"start":[249.35,-79.85,90.38,-100.50,-90.06,-110.40],"pick":[249.33,-75.75,100.63,-114.85,-89.97,-110.45]},
            "2-4":{"start":[253.09,-62.55,66.25,-93.71,-89.88,-106.70],"pick":[253.09,-59.97,77.20,-107.24,-89.83,-106.77]},
            "3-1":{"start":[240.19,-85.99,96.76,-100.47,-89.94,-119.65],"pick":[240.22,-81.69,107.36,-115.48,-90.01,-119.60]},
            "3-3":{"start":[245.91,-69.16,75.62,-96.42,-89.90,-114.03],"pick":[245.91,-65.99,86.15,-110.08,-89.90,-113.99]},
        }

        # ---- fallback 안정화 파라미터 ----
        self.fallback_stable = {"x": None, "y": None, "t": 0.0}
        self.FALLBACK_STABLE_TIME = 2.0      # 2초 유지
        self.FALLBACK_TOLERANCE = 20.0       # ±20 px
        self.FALLBACK_MATCH_TOLERANCE = 25.0 # red/orange와 ±25 px

        # ---- 초기 상태 ----
        self.state = 1
        rospy.loginfo("🧠 ASM FSM Node Ready. Start at state 1.")




    def open_stopper(self):
        rospy.logerr(">>> open_stopper() CALLED NOW <<<")

        if not self.arduino:
            rospy.logwarn("Arduino not connected")
            return

        # 1) Arduino auto-reset 방지 (UNO)
        try:
            self.arduino.setDTR(False)
            self.arduino.setRTS(False)
        except:
            pass

        # 2) 입력 버퍼 비우기
        self.arduino.reset_input_buffer()

        # 3) 명령 즉시 보내기
        self.arduino.write(b'c\n')
        self.arduino.flush()

        rospy.loginfo("[WRITE] c")

        # 4) Arduino 응답 빠르게 읽기 (0.3초) → pytime 사용! (충돌 방지)
        end = pytime.time() + 0.3
        while pytime.time() < end:
            if self.arduino.in_waiting:
                line = self.arduino.readline().decode(errors='ignore').strip()
                rospy.loginfo(f"[ARDUINO] {line}")
                



    # --------- 콜백 ---------
    def cb_red(self, msg: Point): self.red_result.update(msg)
    def cb_orange(self, msg: Point): self.orange_result.update(msg)
    def cb_darkgrid(self, msg: String): self.darkgrid_result.update(msg)
    def cb_gray(self, msg: String): self.gray_result.update(msg)
    def cb_inv_gray(self, msg: String): self.inv_gray_result.update(msg)
    def cb_inv_grid(self, msg: String): self.inv_grid_result.update(msg)
    def cb_fallback_circle(self, msg: Point): self.fallback_circle.update(msg)
    def cb_grid1_fallback(self, msg: Point): self.grid1_fallback.update(msg)
    def cb_grid2_fallback(self, msg: Point): self.grid2_fallback.update(msg)


    # --------- 헬퍼 ---------
    def jdeg(self, arr_deg):
        return [x*self.D2R for x in arr_deg]

    def goJ(self, arr_deg, label=""):
        if label: rospy.loginfo(f"➡️ MoveJ: {label}")
        ok = self.ur5e.go_to_joint_abs(self.jdeg(arr_deg))
        rospy.sleep(1.5)
        return ok
    
    def wait_until_rpy_sign_ok(self, tol_check_rate=0.5):
        while not rospy.is_shutdown():
            try:
                # 현재 pose 읽기
                pose = self.ur5e.manipulator.get_current_pose().pose
                q = pose.orientation
                roll, pitch, yaw = np.degrees(euler_from_quaternion([q.x, q.y, q.z, q.w]))

                # 부호 확인
                roll_ok  = roll < 0
                pitch_ok = pitch > 0
                yaw_ok   = yaw > 0

                if roll_ok and pitch_ok and yaw_ok:
                    rospy.loginfo(f"✅ RPY OK: [R={roll:.3f}, P={pitch:.3f}, Y={yaw:.3f}]")
                    break
                else:
                    rospy.logwarn(
                        f"⚠️  RPY sign mismatch! [R={roll:.3f}, P={pitch:.3f}, Y={yaw:.3f}] → waiting..."
                    )

            except Exception as e:
                rospy.logwarn(f"⚠️  Failed to read RPY: {e}")

            rospy.sleep(tol_check_rate)

    # ✅ 감지 유효 자세 확인 (OBS1 or OBS2)
    def valid_pose_for_detection(self, tol_deg=1.0):
        """
        현재 UR5e 조인트가 감지 가능한 관찰 자세(OBS1/OBS2)인지 확인.
        tol_deg: 허용 오차 (deg)
        """
        try:
            # 현재 UR5e 조인트 읽기 (deg 변환)
            current_joints = [x / self.D2R for x in self.ur5e.manipulator.get_current_joint_values()]
        except Exception as e:
            rospy.logwarn(f"⚠️ Cannot read current joint state: {e}")
            return False

        def is_close(target):
            return all(abs(a - b) <= tol_deg for a, b in zip(current_joints, target))

        # ✅ OBS1 또는 OBS2 근처에 있을 때만 감지 허용
        valid = (
            is_close(self.ASM_OBS_START_JOINTS) or 
            is_close(self.ASM_RED_DETECT_TRIGGER_JOINTS)
        )

        if not valid:
            rospy.loginfo("🚫 Detection ignored (not in OBS1 or OBS2 pose)")
        else:
            rospy.loginfo("✅ Valid detection pose confirmed.")
        return valid


    def grip_on_6s(self):
        self.ur5e.grip_on()
        rospy.sleep(max(0.0, self.GRIP_HOLD - 1.5))

    def grip_off_6s(self):
        self.ur5e.grip_off()
        rospy.sleep(max(0.0, self.GRIP_HOLD - 1.5))

    def point_detected(self, latch: TopicLatch):
        # ✅ OBS1 또는 OBS2에서만 감지 유효
        if not self.valid_pose_for_detection():
            return False
        msg = latch.get()
        return (msg is not None) and (getattr(msg, "x", 0.0) != 0.0)


    def choose_cell_from_string(self, s):
        if not s: return None
        txt = s.strip()
        if "," in txt: txt = txt.split(",")[0].strip()
        if " " in txt: txt = txt.split()[0].strip()
        return txt

    def pick_cycle(self, pose_dict, cell_key, grip_mode="off"):
        if not cell_key or cell_key not in pose_dict:
            rospy.logwarn(f"⚠️ invalid grid key: {cell_key}")
            return False
        p = pose_dict[cell_key]
        ok  = self.goJ(p["start"], f"{cell_key} start")
        ok &= self.goJ(p["pick"],  f"{cell_key} pick")
        if grip_mode == "on": self.grip_on_6s()
        else:                 self.grip_off_6s()
        ok &= self.goJ(p["start"], f"{cell_key} back")
        return ok

    # --------- fallback 안정 감지 (±20px/2s + red/orange 매칭 ±25px) ---------
    # --------- fallback 안정 감지 (±20px/2s + red/orange 매칭 ±25px) ---------
    def fallback_stable_detected(self):
        if not self.fallback_circle.alive():
            rospy.loginfo("⚠️ No active /fallback_circle/result messages.")
            self.fallback_stable["x"] = None
            return False

        msg_fb = self.fallback_circle.get()
        msg_red = self.red_result.get()
        msg_orange = self.orange_result.get()
        if msg_fb is None:
            rospy.loginfo("⚠️ fallback_circle message is None.")
            return False

        # ✅ (0, 0, 0) 퍼블리시 필터링
        if msg_fb.x == 0.0 and msg_fb.y == 0.0 and msg_fb.z == 0.0:
            rospy.logdebug("⚠️ fallback_circle published (0,0,0) → skip.")
            self.fallback_stable["x"] = None
            return False

        x, y = msg_fb.x, msg_fb.y
        now = time()
        ...


        # 🔸 3️⃣ 첫 감지 → 기준 좌표 등록
        if self.fallback_stable["x"] is None:
            self.fallback_stable = {"x": x, "y": y, "t": now}
            return False

        dx = abs(x - self.fallback_stable["x"])
        dy = abs(y - self.fallback_stable["y"])

        # 🔸 4️⃣ ±20px 이내 이동 유지 확인
        stable_ok = (dx <= self.FALLBACK_TOLERANCE and dy <= self.FALLBACK_TOLERANCE)
        time_ok = (now - self.fallback_stable["t"] >= self.FALLBACK_STABLE_TIME)

        # 🔸 5️⃣ 좌표가 새로 벗어났으면 기준 갱신
        if not stable_ok:
            self.fallback_stable = {"x": x, "y": y, "t": now}
            return False

        # 🔸 6️⃣ red/orange 감지와 ±25px 매칭 여부 확인
        def close_enough(msg_ref):
            if msg_ref is None: return False
            return (abs(x - msg_ref.x) <= self.FALLBACK_MATCH_TOLERANCE and
                    abs(y - msg_ref.y) <= self.FALLBACK_MATCH_TOLERANCE)

        match_ok = close_enough(msg_red) or close_enough(msg_orange)

        # 🔸 7️⃣ 모든 조건 충족 시 감지 성공
        if stable_ok and time_ok and match_ok:
            rospy.loginfo(f"✅ Fallback stabilized & matched (x={x:.1f}, y={y:.1f})")
            return True

        return False


    # --------- 메인 루프 ---------
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                if   self.state == 1:   self.state_1()
                elif self.state == 2:   self.state_2()
                elif self.state == 3:   self.state_3()
                elif self.state == 4:   self.state_4()
                elif self.state == 5:   self.state_5()
                elif self.state == 6:   self.state_6()
                elif self.state == 101: self.state_101()
                elif self.state == 102: self.state_102()
                elif self.state == 103: self.state_103()
                elif self.state == 104: self.state_104()
                elif self.state == 105: self.state_105()
                elif self.state == 106: self.state_106()
                elif self.state == 201: self.state_201()
                elif self.state == 202: self.state_202()
                elif self.state == 301: self.state_301()
                elif self.state == 302: self.state_302()
                else:
                    rospy.logwarn(f"❓ Unknown state {self.state} -> 1")
                    self.state = 1
            except Exception as e:
                rospy.logerr(f"💥 Exception in state {self.state}: {e}")
            rate.sleep()

    def state_1(self):
        try:
            # ① 관찰 자세로 이동
            self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS")
            rospy.sleep(1.0)

            # ② 감지 결과 확인
            has_red    = self.point_detected(self.red_result)
            has_orange = self.point_detected(self.orange_result)
            msg_orange = self.orange_result.get()

            # ============================================================
            # 🍊 오렌지 감지 시 — 좌표 기반 픽업 절차 실행
            # ============================================================
            if has_orange and msg_orange:
                dx = msg_orange.x
                dy = msg_orange.y

                # 비정상 토픽 값 (픽셀 좌표 등) 필터링
                if abs(dx) > 200 or abs(dy) > 200:
                    rospy.logwarn(f"⚠️ Ignored abnormal orange detection: x={dx:.1f}, y={dy:.1f}")
                    return

                # -------------------------------
                # EMA 적용 (좌표 안정화 필터)
                # -------------------------------
                if self.prev_orange is None:
                    self.prev_orange = np.array([dx, dy], dtype=float)
                    self.ema_stable_count = 0
                    rospy.loginfo("📊 EMA 초기화 중...")
                    return

                curr = np.array([dx, dy], dtype=float)
                self.prev_orange = (1 - self.ema_alpha) * self.prev_orange + self.ema_alpha * curr
                diff = np.linalg.norm(curr - self.prev_orange)

                if diff < self.ema_stable_thresh_mm:
                    self.ema_stable_count += 2
                else:
                    self.ema_stable_count = 0  # 다시 초기화

                rospy.loginfo_throttle(1.0, f"📈 EMA 안정도: diff={diff:.2f}mm, count={self.ema_stable_count}")

                # 충분히 안정화되었을 때만 픽업 시작
                if self.ema_stable_count < self.ema_min_frames:
                    rospy.loginfo("⏸ EMA 안정화 대기 중...")
                    return

                rospy.loginfo("✅ EMA 안정화 완료 → 픽업 시작")

                dx_m = self.prev_orange[0] / 1000.0
                dy_m = self.prev_orange[1] / 1000.0

                # 1️⃣ XY 평면 이동
                rospy.loginfo(f"➡️ Move to target: dx={dx_m:.4f} m, dy={dy_m:.4f} m")
                self.ur5e.go_to_pose_rel([dx_m, dy_m, 0.0], [0,0,0])
                rospy.sleep(0.3)

                # 2️⃣ Z 하강
                rospy.loginfo("⬇️ Descend 70 mm (Z only)")
                self.ur5e.go_to_pose_rel([0.0, 0.0, -0.07], [0,0,0])

                # 3️⃣ Grip ON
                self.grip_on_6s()

                # 4️⃣ Lift
                self.ur5e.go_to_pose_rel([0.0, 0.0, 0.115], [0,0,0])

                # 5️⃣ 관찰 자세 복귀
                self.goJ(self.ASM_OBS_GRID_JOINTS, "ASM_OBS_GRID_JOINTS")

                # 6️⃣ 다음 상태 전환
                self.state = 2
                rospy.loginfo("➡️ state 2")

            elif has_red and not has_orange:
                self.state = 3
                rospy.loginfo("➡️ state 3")

            else:
                rospy.loginfo("⏸ state1 hold")

        except Exception as e:
            rospy.logerr(f"💥 Exception in state 1: {e}")



    def state_2(self):
        msg = self.darkgrid_result.get()
        cell = self.choose_cell_from_string(msg.data if msg else "")
        if cell:
            self.pick_cycle(self.DARKGRID_POSES, cell, grip_mode="off")
        else:
            rospy.loginfo("⏸ state2 no grid")
        self.state = 3
        rospy.loginfo("↩️  back to state 3")

    def state_3(self):
        try:
            # ① 관찰 자세로 이동
            self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")
            rospy.sleep(0.5)  # 관찰 자세 안정화

            # 감지 결과 읽기
            has_red    = self.point_detected(self.red_result)
            has_orange = self.point_detected(self.orange_result)
            msg_orange = self.orange_result.get()

            # ============================================================
            # 🍊 오렌지 감지 시 — state 1과 동일한 좌표 기반 픽업 절차
            # ============================================================
            if has_orange and msg_orange:
                dx = msg_orange.x / 1000.0   # mm → m 변환
                dy = msg_orange.y / 1000.0

                # 비정상 좌표 필터링
                if abs(msg_orange.x) > 200 or abs(msg_orange.y) > 200:
                    rospy.logwarn(f"⚠️ Ignored abnormal orange detection: x={msg_orange.x:.1f}, y={msg_orange.y:.1f}")
                    return

                rospy.loginfo(f"🍊 Orange detected (state3): dx={dx:.4f} m, dy={dy:.4f} m")

                # 1️⃣ XY 평면 이동
                rospy.loginfo("➡️ Moving in XY plane first...")
                self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0.0, 0.0, 0.0])
                rospy.sleep(0.3)

                # 2️⃣ Z 하강 −0.07 m
                rospy.loginfo("⬇️ Descend 70 mm (Z only)")
                self.ur5e.go_to_pose_rel([0.0, 0.0, -0.07], [0.0, 0.0, 0.0])

                # 3️⃣ Grip ON
                rospy.loginfo("✊ Gripper ON")
                self.grip_on_6s()

                # 4️⃣ Z 상승 +0.115 m
                rospy.loginfo("⬆️ Lift up 115 mm (Z only)")
                self.ur5e.go_to_pose_rel([0.0, 0.0, 0.115], [0.0, 0.0, 0.0])

                # 5️⃣ 관찰 자세 복귀
                self.goJ(self.ASM_OBS_GRID_JOINTS, "ASM_OBS_GRID_JOINTS")

                # 6️⃣ 다음 상태로
                self.state = 4
                rospy.loginfo("➡️ state 4")

            # ============================================================
            # 🔴 오렌지 감지 X, 빨간 원만 감지됨 → state 5로 전환
            # ============================================================
            elif has_red and not has_orange:
                self.state = 5
                rospy.loginfo("➡️ state 5")

            # ============================================================
            # 🚫 감지 안됐을 때
            # ============================================================
            else:
                rospy.loginfo("⏸ state3 hold")

        except Exception as e:
            rospy.logerr(f"💥 Exception in state 3: {e}")


    def state_4(self):
        msg = self.darkgrid_result.get()
        cell = self.choose_cell_from_string(msg.data if msg else "")
        if cell:
            self.pick_cycle(self.DARKGRID_POSES, cell, grip_mode="off")
        else:
            rospy.loginfo("⏸ state4 no grid")
        self.state = 5
        rospy.loginfo("➡️  state 5")


    def state_5(self):
        rospy.loginfo("➡️ Enter state_5()")

        # ① 관찰자세로 이동 (그리드 전체 관찰)
        self.goJ(self.OBS_GRID_JOINTS, "ASM_OBS_GRID_JOINTS")
        rospy.sleep(0.5)

        # ② /gray_detection/result 감지 시도
        rospy.loginfo("🕐 Waiting for /gray_detection/result ...")
        gray_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
            msg = self.gray_result.get()
            if msg and msg.data and msg.data.lower() != "none":
                gray_msg = msg
                break
            rospy.sleep(0.1)

        if not gray_msg:
            rospy.logwarn("❌ No gray cell detected → return to OBS_START_JOINTS → state101")
            self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS (after gray fail)")
            self.state = 101
            return

        # ✅ 감지 성공 시 셀 이름 결정
        cell = self.choose_cell_from_string(gray_msg.data if gray_msg else "")
        if not cell or cell not in self.ASM_GRID_POSES:
            rospy.logwarn(f"⚠️ Invalid or unknown gray cell: {gray_msg.data}")
            self.goJ(self.ASM_OBS_START_JOINTS, "OBS_START_JOINTS (invalid gray)")
            self.state = 101
            return

        rospy.loginfo(f"✅ Gray cell detected: {cell}")

        # ③ 해당 셀의 START 자세로 이동
        start_pose = self.ASM_GRID_POSES[cell]["start"]
        self.goJ(start_pose, f"{cell} start (state5)")
        rospy.sleep(0.3)

        # ④ /grid1_fallback/result 감지 시도
        rospy.loginfo("🕐 Waiting for /grid1_fallback/result ...")
        fb_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
            msg = self.grid1_fallback.get()
            if msg and (msg.x != 0.0 or msg.y != 0.0):
                fb_msg = msg
                break
            rospy.sleep(0.05)

        if fb_msg:
            dx = fb_msg.x / 1000.0  # mm → m
            dy = fb_msg.y / 1000.0
            rospy.loginfo(f"📍 grid1_fallback offset: dx={dx:.3f} m, dy={dy:.3f} m")

            self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
            rospy.sleep(0.3)
            self.ur5e.go_to_pose_rel([0, 0, -0.115], [0, 0, 0])
            self.grip_on_6s()
            self.ur5e.go_to_pose_rel([0, 0, 0.115], [0, 0, 0])
        else:
            rospy.logwarn("⚠️ No /grid1_fallback/result → skipping pre-pick")

        # ⑤ RED DETECTION으로 미세 조정
        self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS (red detection)")
        rospy.loginfo("⏸ Waiting 1.5s for camera stabilization...")
        rospy.sleep(0.5)

        rospy.loginfo("🕐 Waiting for /red_detection/result ...")
        red_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 3.0 and not rospy.is_shutdown():
            msg = self.red_result.get()
            if msg and msg.x != 0.0 and msg.y != 0.0:
                red_msg = msg
                rospy.loginfo("🕐 Red detected — waiting 1.5s for EMA stabilization...")
                rospy.sleep(0.5)   # ✅ EMA가 평균값으로 안정화될 시간을 확보
                break
            rospy.sleep(0.1)

        if not red_msg:
            rospy.logwarn("❌ No /red_detection/result → state101")
            self.state = 101
            return

        dx = red_msg.x / 1000.0
        dy = red_msg.y / 1000.0
        rospy.loginfo(f"📍 Red offset: dx={dx:.3f} m, dy={dy:.3f} m")

        try:
            rospy.loginfo(f"🚀 RELATIVE MOVE START: X={dx:.4f}m, Y={dy:.4f}m, Z=0.0m")
            self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
            rospy.sleep(0.3)
        except Exception as e:
            rospy.logwarn(f"⚠️ XY offset move failed: {e}")

        # ⑥ 배치 및 복귀
        self.ur5e.go_to_pose_rel([0, 0, -0.04], [0, 0, 0])
        self.grip_off_6s()
        self.ur5e.go_to_pose_rel([0, 0, 0.04], [0, 0, 0])

        # ⑦ 복귀 및 다음 단계
        self.goJ(start_pose, f"{cell} return (state5)")
        self.state = 6
        rospy.loginfo("➡️ Next: state 6")


    # ----------------------------------------------------------
    def state_6(self):
        rospy.loginfo("➡️ Enter state_6()")

        # ① 관찰자세 이동 (그리드 전체 관찰)
        self.goJ(self.OBS_GRID_JOINTS, "ASM_OBS_GRID_JOINTS")
        rospy.sleep(0.5)

        # ② /gray_detection/result 감지 시도
        rospy.loginfo("🕐 Waiting for /gray_detection/result ...")
        gray_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
            msg = self.gray_result.get()
            if msg and msg.data and msg.data.lower() != "none":
                gray_msg = msg
                break
            rospy.sleep(0.1)

        # ⚠️ 감지 실패 시 → state 201
        if not gray_msg:
            rospy.logwarn("❌ No gray cell detected → return to OBS_START_JOINTS → state201")
            self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS (after gray fail)")

            ###### ⭐ 여기 추가 ######
            rospy.sleep(1.0)
            self.open_stopper()
            rospy.sleep(1.0)
            ###### ⭐ 여기까지 ######

            self.state = 201
            return

        # ✅ 감지 성공 시 셀 이름 결정
        cell = self.choose_cell_from_string(gray_msg.data if gray_msg else "")
        if not cell or cell not in self.ASM_GRID_POSES:
            rospy.logwarn(f"⚠️ Invalid or unknown gray cell: {gray_msg.data}")
            self.goJ(self.ASM_OBS_START_JOINTS, "OBS_START_JOINTS (invalid gray)")

            ###### ⭐ 여기 추가 ######
            rospy.sleep(1.0)
            self.open_stopper()
            rospy.sleep(1.0)
            ###### ⭐ 여기까지 ######

            self.state = 201
            return

        rospy.loginfo(f"✅ Gray cell detected: {cell}")

        # ③ 해당 셀의 START 자세로 이동
        start_pose = self.ASM_GRID_POSES[cell]["start"]
        self.goJ(start_pose, f"{cell} start (state6)")
        rospy.sleep(0.3)

        # ④ /grid1_fallback/result 감지 시도
        rospy.loginfo("🕐 Waiting for /grid1_fallback/result ...")
        fb_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
            msg = self.grid1_fallback.get()
            if msg and (msg.x != 0.0 or msg.y != 0.0):
                fb_msg = msg
                break
            rospy.sleep(0.05)

        if fb_msg:
            # ⚠️ 단위만 m로 변환 — 축변환 금지
            dx = fb_msg.x / 1000.0
            dy = fb_msg.y / 1000.0
            rospy.loginfo(f"📍 grid1_fallback offset: dx={dx:.3f} m, dy={dy:.3f} m")

            self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
            rospy.sleep(1.0)
            self.ur5e.go_to_pose_rel([0, 0, -0.115], [0, 0, 0])
            self.grip_on_6s()
            self.ur5e.go_to_pose_rel([0, 0, 0.115], [0, 0, 0])
        else:
            rospy.logwarn("⚠️ No /grid1_fallback/result → skipping pre-pick")

        # ⑤ RED DETECTION 기반 미세조정
        self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")
        rospy.loginfo("⏸ Waiting 1.5s for camera stabilization...")
        rospy.sleep(0.5)

        rospy.loginfo("🕐 Waiting for /red_detection/result ...")
        red_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 3.0 and not rospy.is_shutdown():
            msg = self.red_result.get()
            if msg and msg.x != 0.0 and msg.y != 0.0:
                red_msg = msg
                rospy.loginfo("🕐 Red detected — waiting 1.5s for EMA stabilization...")
                rospy.sleep(0.5)   # ✅ EMA가 평균값으로 안정화될 시간을 확보
                break
            rospy.sleep(0.1)

        # ⚠️ red 감지 실패 시 → state201
        if not red_msg:
            rospy.logwarn("❌ No /red_detection/result → state201")

            ###### ⭐ 여기 추가 ######
            rospy.sleep(1.0)
            self.open_stopper()
            rospy.sleep(1.0)
            ###### ⭐ 여기까지 ######
            
            self.state = 201
            return

        dx = red_msg.x / 1000.0
        dy = red_msg.y / 1000.0
        rospy.loginfo(f"📍 Red offset: dx={dx:.3f} m, dy={dy:.3f} m")

        try:
            rospy.loginfo(f"🚀 RELATIVE MOVE START: X={dx:.4f}m, Y={dy:.4f}m, Z=0.0m")
            self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
            rospy.sleep(0.3)
        except Exception as e:
            rospy.logwarn(f"⚠️ XY offset move failed: {e}")

        # ⑥ Grip off
        self.ur5e.go_to_pose_rel([0, 0, -0.04], [0, 0, 0])
        self.grip_off_6s()
        self.ur5e.go_to_pose_rel([0, 0, 0.04], [0, 0, 0])

        # ⑦ 종료 및 다음 단계
        rospy.loginfo("✅ state_6 complete → move to state 1")

        ###### ⭐ 여기 추가 ######
        rospy.sleep(1.0)
        self.open_stopper()
        rospy.sleep(1.0)
        ###### ⭐ 여기까지 ######

        self.state = 1


    def state_101(self):
        rospy.loginfo("➡️ Enter state_101()")

        # ✅ 시작 포즈로 이동
        self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS")
        rospy.sleep(1.0)
        has_red = self.point_detected(self.red_result)
        if has_red:
            rospy.loginfo("🔴 Red detected immediately → state 103")
            self.state = 103
            rospy.loginfo("➡️ state 103")
            return   # ← 이후 로직 절대 실행되지 않게

        # ------------------------------------------------------------
        # ② 감지 확인 (노이즈 방어 포함)
        # ------------------------------------------------------------
        rospy.loginfo("🕐 Waiting for /grid1_fallback/result ...")
        fb_msg = None
        recent_points = []
        t0 = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
            msg = self.grid1_fallback.get()
            if not msg or (msg.x == 0.0 and msg.y == 0.0):
                rospy.sleep(0.05)
                continue

            if abs(msg.x) > 200 or abs(msg.y) > 200:
                rospy.logwarn(f"⚠️ Ignored abnormal fallback detection: x={msg.x:.1f}, y={msg.y:.1f}")
                continue

            recent_points.append((msg.x, msg.y))
            if len(recent_points) > 15:
                recent_points.pop(0)

            if len(recent_points) >= 3:
                xs, ys = zip(*recent_points)
                std_x, std_y = np.std(xs), np.std(ys)
                if std_x < 2.0 and std_y < 2.0:  # mm 기준 안정 조건
                    fb_msg = msg
                    rospy.loginfo("✅ Stable fallback detection confirmed.")
                    break

            rospy.sleep(0.05)

        has_red = self.point_detected(self.red_result)
        has_gray_circle = fb_msg is not None

        # ------------------------------------------------------------
        # 🩶 회색 원 감지 성공 (EMA 기반 안정화 포함)
        # ------------------------------------------------------------
        if has_gray_circle:
            rospy.loginfo("🩶 Gray circle detected — starting EMA stabilization check")

            rospy.sleep(0.5)

        # ✅ 최신 감지값으로 보정
        msg = self.grid1_fallback.get()
        if msg and (msg.x != 0.0 or msg.y != 0.0):
            fb_msg = msg
            rospy.loginfo("🔄 Updated fallback position after delay.")

            last_points = []
            stable = False
            start_time = rospy.Time.now().to_sec()

            # EMA 안정화 루프
            while not rospy.is_shutdown():
                msg = self.grid1_fallback.get()
                if msg and (msg.x != 0.0 or msg.y != 0.0):
                    last_points.append((msg.x, msg.y))
                    if len(last_points) > 5:
                        last_points.pop(0)

                    if len(last_points) >= 3:
                        diffs = []
                        for i in range(1, len(last_points)):
                            dx = last_points[i][0] - last_points[i - 1][0]
                            dy = last_points[i][1] - last_points[i - 1][1]
                            diffs.append((dx**2 + dy**2) ** 0.5)
                        avg_move = sum(diffs) / len(diffs)
                        rospy.loginfo_throttle(1.0, f"📉 EMA Δ movement avg={avg_move:.2f} mm")

                        if avg_move < 3.0:  # 1mm 이하 변화 → 안정화 완료
                            stable = True
                            break

                if rospy.Time.now().to_sec() - start_time > 6.0:
                    rospy.logwarn("⏱ EMA stabilization timeout — proceeding anyway")
                    break

                rospy.sleep(0.1)

            if stable:
                rospy.loginfo("✅ EMA stabilized — executing pickup")
            else:
                rospy.logwarn("⚠️ Proceeding without full EMA stabilization")

            dx = fb_msg.x / 1000.0
            dy = fb_msg.y / 1000.0
            rospy.loginfo(f"📍 Gray circle offset: dx={dx:.3f} m, dy={dy:.3f} m")

            rospy.sleep(0.3)

            # XY 이동 → Z 하강 → Grip → 상승
            self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
            rospy.sleep(0.3)
            self.ur5e.go_to_pose_rel([0, 0, -0.07], [0, 0, 0])
            self.grip_on_6s()
            self.ur5e.go_to_pose_rel([0, 0, 0.115], [0, 0, 0])

            # 다음 단계
            self.goJ(self.OBS_GRID_JOINTS, "OBS_GRID_JOINTS")
            rospy.sleep(0.3)
            self.state = 102
            rospy.loginfo("➡️ state 102")

        # ------------------------------------------------------------
        # 🔴 빨간 원 감지 시 (EMA 없이 즉시)
        # ------------------------------------------------------------
        elif has_red and not has_gray_circle:
            rospy.loginfo("⚠️ No gray circle, but red detected → state 103")
            self.state = 103
            rospy.loginfo("➡️ state 103")

        # ------------------------------------------------------------
        # 🚫 아무 감지도 없음
        # ------------------------------------------------------------
        else:
            rospy.loginfo("⏸ state101 hold")


    def state_102(self):
        msg = self.inv_gray_result.get()
        cell = self.choose_cell_from_string(msg.data if msg else "")
        if cell:
            self.pick_cycle(self._FALLBACK_POSES, cell, grip_mode="off")
        else:
            rospy.loginfo("⏸ state102 no inv-gray")
        self.state = 103
        rospy.loginfo("➡️  state 103")

    def state_103(self):
        rospy.loginfo("➡️ Enter state_103()")

        # ✅ 감지 포즈 이동
        self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")
        has_red = self.point_detected(self.red_result)
        if has_red:
            rospy.loginfo("🔴 Red detected immediately → state 103")
            self.state = 105
            rospy.loginfo("➡️ state 103")
            return   # ← 이후 로직 절대 실행되지 않게

        # ------------------------------------------------------------
        # ② 감지 확인
        # ------------------------------------------------------------
        rospy.loginfo("🕐 Waiting for /grid1_fallback/result ...")
        fb_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
            msg = self.grid1_fallback.get()
            if msg and (msg.x != 0.0 or msg.y != 0.0):
                fb_msg = msg
                break
            rospy.sleep(0.05)

        has_red = self.point_detected(self.red_result)
        has_gray_circle = fb_msg is not None  # grid1_fallback 감지 성공 시 True

        # ------------------------------------------------------------
        # 🩶 회색 원 감지 성공 (grid1_fallback + EMA 안정화)
        # ------------------------------------------------------------
        if has_gray_circle:
            rospy.loginfo("🩶 Gray circle detected — starting EMA stabilization check")

            rospy.sleep(0.5)

        # ✅ 최신 감지값으로 보정
        msg = self.grid1_fallback.get()
        if msg and (msg.x != 0.0 or msg.y != 0.0):
            fb_msg = msg
            rospy.loginfo("🔄 Updated fallback position after delay.")

            last_points = []
            stable = False
            start_time = rospy.Time.now().to_sec()

            # EMA 안정화 루프
            while not rospy.is_shutdown():
                msg = self.grid1_fallback.get()
                if msg and (msg.x != 0.0 or msg.y != 0.0):
                    last_points.append((msg.x, msg.y))
                    if len(last_points) > 5:
                        last_points.pop(0)

                    if len(last_points) >= 3:
                        diffs = []
                        for i in range(1, len(last_points)):
                            dx = last_points[i][0] - last_points[i - 1][0]
                            dy = last_points[i][1] - last_points[i - 1][1]
                            diffs.append((dx**2 + dy**2) ** 0.5)
                        avg_move = sum(diffs) / len(diffs)
                        rospy.loginfo_throttle(1.0, f"📉 EMA Δ movement avg={avg_move:.2f} mm")

                        if avg_move < 3.0:  # 1mm 이하 변화 → 안정화 완료
                            stable = True
                            break

                if rospy.Time.now().to_sec() - start_time > 6.0:
                    rospy.logwarn("⏱ EMA stabilization timeout — proceeding anyway")
                    break

                rospy.sleep(0.1)

            if stable:
                rospy.loginfo("✅ EMA stabilized — executing pickup")
            else:
                rospy.logwarn("⚠️ Proceeding without full EMA stabilization")

            dx = fb_msg.x / 1000.0
            dy = fb_msg.y / 1000.0
            rospy.loginfo(f"📍 Gray circle offset: dx={dx:.3f} m, dy={dy:.3f} m")

            # 1️⃣ XY 평면 이동
            rospy.loginfo("➡️ Moving in XY plane first...")
            self.ur5e.go_to_pose_rel(
                relative_xyz=[dx, dy, 0.0],
                relative_rpy=[0.0, 0.0, 0.0]
            )

            # 2️⃣ 안정화 대기
            rospy.sleep(0.5)

            # 3️⃣ Z축 하강
            rospy.loginfo("⬇️ Descend 70 mm (Z only)")
            self.ur5e.go_to_pose_rel(
                relative_xyz=[0.0, 0.0, -0.07],
                relative_rpy=[0.0, 0.0, 0.0]
            )

            # 4️⃣ Grip ON
            rospy.loginfo("✊ Gripper ON")
            self.grip_on_6s()

            # 5️⃣ 상승
            rospy.loginfo("⬆️ Lift up 115 mm (Z only)")
            self.ur5e.go_to_pose_rel(
                relative_xyz=[0.0, 0.0, 0.115],
                relative_rpy=[0.0, 0.0, 0.0]
            )

            # 6️⃣ 관찰 자세로 복귀 및 다음 단계 전환
            self.goJ(self.OBS_GRID_JOINTS, "OBS_GRID_JOINTS")
            rospy.sleep(0.3)
            self.state = 104
            rospy.loginfo("↩️  state 102")

        # ------------------------------------------------------------
        # 🔴 회색 원(X), 빨간 원(O) → state 105
        # ------------------------------------------------------------
        elif has_red and not has_gray_circle:
            rospy.loginfo("⚠️ No gray circle, but red detected → state 105")
            self.state = 105
            rospy.loginfo("➡️  state 105")

        # ------------------------------------------------------------
        # 🚫 아무 감지도 없음 → hold
        # ------------------------------------------------------------
        else:
            rospy.loginfo("⏸ state103 hold")



    def state_104(self):
        msg = self.inv_gray_result.get()
        cell = self.choose_cell_from_string(msg.data if msg else "")
        if cell:
            self.pick_cycle(self._FALLBACK_POSES, cell, grip_mode="off")
        else:
            rospy.loginfo("⏸ state104 no inv-gray")
        self.state = 105
        rospy.loginfo("➡️  state 105")

    def state_105(self):
        rospy.loginfo("➡️ Enter state_105()")

        # ① 관찰 포즈 이동
        self.goJ(self.ASM_OBS_GRID_JOINTS, "ASM_OBS_GRID_JOINTS")
        rospy.sleep(0.5)

        # ② inverse grid 감지 시도
        rospy.loginfo("🕐 Waiting for /inverse_grid_detection_result ...")
        inv_grid_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
            msg = self.inv_grid_result.get()
            if msg and msg.data and msg.data.lower() != "none":
                inv_grid_msg = msg
                break
            rospy.sleep(0.1)

        if inv_grid_msg:
            cell = inv_grid_msg.data.strip()
            rospy.loginfo(f"✅ Inverse grid cell detected: {cell}")

            if cell in self.DIS_GRID_POSES:
                start_pose = self.DIS_GRID_POSES[cell]["start"]
                self.goJ(start_pose, f"{cell} start (inverse grid)")
                rospy.sleep(0.5)

                # ③ grid1_fallback 상대좌표 보정
                rospy.loginfo("🕐 Waiting for /grid1_fallback/result ...")
                fb_msg = None
                t0 = rospy.Time.now().to_sec()
                while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
                    msg = self.grid2_fallback.get()
                    if msg and (msg.x != 0.0 or msg.y != 0.0):
                        fb_msg = msg
                        break
                    rospy.sleep(0.05)

                if fb_msg:
                    dx = fb_msg.x / 1000.0
                    dy = fb_msg.y / 1000.0
                    rospy.loginfo(f"📍 grid1_fallback offset: dx={dx:.3f} m, dy={dy:.3f} m")

                    self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
                    rospy.sleep(0.3)
                    self.ur5e.go_to_pose_rel([0, 0, -0.115], [0, 0, 0])
                    self.grip_on_6s()
                    self.ur5e.go_to_pose_rel([0, 0, 0.115], [0, 0, 0])
                else:
                    rospy.logwarn("⚠️ No /grid1_fallback/result → skipping pre-pick")

                # ④ ASM_OBS_START_JOINTS로 복귀 후 red detection 수행
                self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS")
                rospy.sleep(0.5)

                rospy.loginfo("🕐 Waiting for /red_detection/result ...")
                red_msg = None
                t0 = rospy.Time.now().to_sec()
                while (rospy.Time.now().to_sec() - t0) < 3.0 and not rospy.is_shutdown():
                    msg = self.red_result.get()
                    if msg and msg.x != 0.0 and msg.y != 0.0:
                        red_msg = msg
                        break
                    rospy.sleep(0.1)

                if red_msg:
                    dx = red_msg.x / 1000.0
                    dy = red_msg.y / 1000.0
                    rospy.loginfo(f"📍 Red offset: dx={dx:.3f} m, dy={dy:.3f} m")

                    try:
                        rospy.loginfo(f"🚀 RELATIVE MOVE START: X={dx:.4f}m, Y={dy:.4f}m, Z=0.0m")
                        self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
                        rospy.sleep(0.3)
                    except Exception as e:
                        rospy.logwarn(f"⚠️ XY offset move failed: {e}")

                    # ⑤ 배치 및 복귀
                    self.ur5e.go_to_pose_rel([0, 0, -0.04], [0, 0, 0])
                    self.grip_off_6s()
                    self.ur5e.go_to_pose_rel([0, 0, 0.04], [0, 0, 0])

                    # ✅ state 전환
                    self.state = 106
                    rospy.loginfo("➡️ state 106")
                else:
                    rospy.logwarn("❌ No /red_detection/result after inverse pick — skipping placement")
                    self.state = 1
                    rospy.loginfo("↩️ Return to state 1 (no red detection)")

            else:
                rospy.logwarn(f"⚠️ Invalid grid cell name: {cell}")
                self.state = 1
                rospy.loginfo("↩️ Return to state 1 (invalid cell)")

        else:
            rospy.logwarn("❌ No inverse grid detection result received")
            self.state = 1
            rospy.loginfo("↩️ Return to state 1 (no inverse grid)")



    def state_106(self):
        rospy.loginfo("➡️ Enter state_106()")

        # ① 관찰 포즈 이동 (inverse grid 감지용)
        self.goJ(self.ASM_OBS_GRID_JOINTS, "ASM_OBS_GRID_JOINTS")
        rospy.sleep(0.5)

        # ② inverse grid 감지 시도
        rospy.loginfo("🕐 Waiting for /inverse_grid_detection_result ...")
        inv_grid_msg = None
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
            msg = self.inv_grid_result.get()
            if msg and msg.data and msg.data.lower() != "none":
                inv_grid_msg = msg
                break
            rospy.sleep(0.1)

        if inv_grid_msg:
            cell = inv_grid_msg.data.strip()
            rospy.loginfo(f"✅ Inverse grid cell detected: {cell}")

            if cell in self.DIS_GRID_POSES:
                start_pose = self.DIS_GRID_POSES[cell]["start"]
                self.goJ(start_pose, f"{cell} start (inverse grid)")
                rospy.sleep(0.5)

                # ③ grid1_fallback 상대좌표 보정
                rospy.loginfo("🕐 Waiting for /grid1_fallback/result ...")
                fb_msg = None
                t0 = rospy.Time.now().to_sec()
                while (rospy.Time.now().to_sec() - t0) < 2.5 and not rospy.is_shutdown():
                    msg = self.grid2_fallback.get()
                    if msg and (msg.x != 0.0 or msg.y != 0.0):
                        fb_msg = msg
                        break
                    rospy.sleep(0.05)

                if fb_msg:
                    dx = fb_msg.x / 1000.0
                    dy = fb_msg.y / 1000.0
                    rospy.loginfo(f"📍 grid1_fallback offset: dx={dx:.3f} m, dy={dy:.3f} m")

                    self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
                    rospy.sleep(0.3)
                    self.ur5e.go_to_pose_rel([0, 0, -0.115], [0, 0, 0])
                    self.grip_on_6s()
                    self.ur5e.go_to_pose_rel([0, 0, 0.115], [0, 0, 0])
                else:
                    rospy.logwarn("⚠️ No /grid1_fallback/result → skipping pre-pick")

                # ④ ASM_RED_DETECT_TRIGGER_JOINTS로 복귀 후 red detection 수행
                self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")
                rospy.sleep(0.5)

                rospy.loginfo("🕐 Waiting for /red_detection/result ...")
                red_msg = None
                t0 = rospy.Time.now().to_sec()
                while (rospy.Time.now().to_sec() - t0) < 3.0 and not rospy.is_shutdown():
                    msg = self.red_result.get()
                    if msg and msg.x != 0.0 and msg.y != 0.0:
                        red_msg = msg
                        break
                    rospy.sleep(0.1)

                if red_msg:
                    dx = red_msg.x / 1000.0
                    dy = red_msg.y / 1000.0
                    rospy.loginfo(f"📍 Red offset: dx={dx:.3f} m, dy={dy:.3f} m")

                    try:
                        rospy.loginfo(f"🚀 RELATIVE MOVE START: X={dx:.4f}m, Y={dy:.4f}m, Z=0.0m")
                        self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0, 0, 0])
                        rospy.sleep(0.5)
                    except Exception as e:
                        rospy.logwarn(f"⚠️ XY offset move failed: {e}")

                    # ⑤ 배치 및 복귀
                    self.ur5e.go_to_pose_rel([0, 0, -0.04], [0, 0, 0])
                    self.grip_off_6s()
                    self.ur5e.go_to_pose_rel([0, 0, 0.04], [0, 0, 0])

                    ###### ⭐ 여기 추가 ######
                    rospy.sleep(1.0)
                    self.open_stopper()
                    rospy.sleep(1.0)
                    ###### ⭐ 여기까지 ######

                    # ✅ state 전환
                    self.state = 101
                    rospy.loginfo("➡️ state 101")
                else:
                    rospy.logwarn("❌ No /red_detection/result after inverse pick — skipping placement")

                    ###### ⭐ 여기 추가 ######
                    rospy.sleep(1.0)
                    self.open_stopper()
                    rospy.sleep(1.0)
                    ###### ⭐ 여기까지 ######

                    self.state = 301
                    rospy.loginfo("↩️ Move to state 301 (no red detection)")

            else:
                rospy.logwarn(f"⚠️ Unknown grid cell name: {cell}")

                ###### ⭐ 여기 추가 ######
                rospy.sleep(1.0)
                self.open_stopper()
                rospy.sleep(1.0)
                ###### ⭐ 여기까지 ######

                self.state = 301
                rospy.loginfo("↩️ Move to state 301 (invalid cell)")

        else:
            # ❌ inverse grid 감지 실패 시
            rospy.logwarn("❌ No inverse grid detection result received")

            ###### ⭐ 여기 추가 ######
            rospy.sleep(1.0)
            self.open_stopper()
            rospy.sleep(1.0)
            ###### ⭐ 여기까지 ######

            self.state = 301
            rospy.loginfo("↩️ Move to state 301 (no inverse grid)")


    def state_201(self):
        rospy.loginfo("➡️ Enter state_201()")

        # ① 시작 자세
        self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS")
        rospy.sleep(3.0)

        # ② 감지
        msg_orange = self.orange_result.get()
        has_orange = self.point_detected(self.orange_result)
        has_red    = self.point_detected(self.red_result)

        # 🍊 오렌지 감지 시 → pick & place → state 202
        if has_orange and msg_orange:

            dx = msg_orange.x
            dy = msg_orange.y

            # 비정상 좌표 방지
            if abs(dx) > 200 or abs(dy) > 200:
                rospy.logwarn(f"⚠️ Ignored abnormal orange detection: x={dx:.1f}, y={dy:.1f}")
                return

            # ============================================================
            # 🔶🔶🔶 ① EMA 초기화 (state_1 그대로)
            # ============================================================
            if self.prev_orange is None:
                self.prev_orange = np.array([dx, dy], dtype=float)
                self.ema_stable_count = 0
                rospy.loginfo("📊 EMA 초기화 중... (state_201)")
                return

            # ============================================================
            # 🔶🔶🔶 ② EMA 적용
            # ============================================================
            curr = np.array([dx, dy], dtype=float)
            self.prev_orange = (1 - self.ema_alpha) * self.prev_orange + self.ema_alpha * curr

            diff = np.linalg.norm(curr - self.prev_orange)

            if diff < self.ema_stable_thresh_mm:
                self.ema_stable_count += 1
            else:
                self.ema_stable_count = 0

            rospy.loginfo_throttle(
                1.0,
                f"📈 (state201) EMA 안정도: diff={diff:.2f}mm, count={self.ema_stable_count}"
            )

            # ============================================================
            # 🔶🔶🔶 ③ EMA 안정화 대기
            # ============================================================
            if self.ema_stable_count < self.ema_min_frames:
                rospy.loginfo("⏸ (state201) EMA 안정화 대기 중…")
                return

            rospy.loginfo("✅ (state201) EMA 안정화 완료 → 픽업 시작")

            # ============================================================
            # 🔶 EMA 안정된 좌표 사용
            # ============================================================
            dx_m = self.prev_orange[0] / 1000.0
            dy_m = self.prev_orange[1] / 1000.0

            # ================== 원본 로직 그대로 유지 ====================
            self.ur5e.go_to_pose_rel([dx_m, dy_m, 0,], [0,0,0])
            rospy.sleep(0.8)
            self.ur5e.go_to_pose_rel([0,0,-0.07],[0,0,0])
            self.grip_on_6s()
            self.ur5e.go_to_pose_rel([0,0,0.115],[0,0,0])

            # Grid 놓기
            self.goJ(self.ASM_OBS_GRID_JOINTS, "ASM_OBS_GRID_JOINTS")
            msg = self.darkgrid_result.get()
            cell = self.choose_cell_from_string(msg.data if msg else "")

            if cell and cell in self.DARKGRID_POSES:
                start_pose = self.DARKGRID_POSES[cell]["start"]
                pick_pose  = self.DARKGRID_POSES[cell]["pick"]
                self.goJ(start_pose, f"{cell} start")
                rospy.sleep(0.5)
                self.goJ(pick_pose, f"{cell} place")
                self.grip_off_6s()
                rospy.sleep(0.5)
                self.goJ(start_pose, f"{cell} return start")
            else:
                rospy.logwarn("❌ No valid dark grid detected.")

            # 다음 단계
            self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")
            self.state = 202
            rospy.loginfo("➡️ Transition to state_202")

        # 🔴 빨간색만 감지
        elif has_red and not has_orange:
            rospy.loginfo("🔴 Red only detected → move to red trigger (state 202)")
            self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")
            self.state = 202

        # 🚫 아무 감지도 없음
        else:
            rospy.loginfo("⏸ state201 hold (no orange/red detected)")




    # ============================================================
    # 🍊 state_202: 두 번째 오렌지 pick + grid 놓기 + state101로 복귀
    # ============================================================
    def state_202(self):
        rospy.loginfo("➡️ Enter state_202()")

        msg_orange2 = self.orange_result.get()
        has_orange2 = self.point_detected(self.orange_result)
        has_red2    = self.point_detected(self.red_result)

        # 🍊 두 번째 오렌지 감지 → pick & place → state101
        if has_orange2 and msg_orange2:
            dx = msg_orange2.x / 1000.0
            dy = msg_orange2.y / 1000.0
            rospy.sleep(2.0)
            rospy.loginfo(f"🍊 (Second) Orange detected: dx={dx:.4f}, dy={dy:.4f}")

            # 픽업
            self.ur5e.go_to_pose_rel([dx, dy, 0], [0,0,0])
            rospy.sleep(0.8)
            self.ur5e.go_to_pose_rel([0,0,-0.07],[0,0,0])
            self.grip_on_6s()
            self.ur5e.go_to_pose_rel([0,0,0.115],[0,0,0])

            # Grid 놓기
            self.goJ(self.ASM_OBS_GRID_JOINTS, "ASM_OBS_GRID_JOINTS")
            msg2 = self.darkgrid_result.get()
            cell2 = self.choose_cell_from_string(msg2.data if msg2 else "")
            if cell2 and cell2 in self.DARKGRID_POSES:
                start_pose2 = self.DARKGRID_POSES[cell2]["start"]
                pick_pose2  = self.DARKGRID_POSES[cell2]["pick"]
                self.goJ(start_pose2, f"{cell2} start")
                rospy.sleep(0.5)
                self.goJ(pick_pose2, f"{cell2} place")
                self.grip_off_6s()
                rospy.sleep(0.5)
                self.goJ(start_pose2, f"{cell2} return start")
            else:
                rospy.logwarn("❌ No valid dark grid detected in state_202.")

            # 완료 후 state101로 복귀
            self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS")

            ###### ⭐ 여기 추가 ######
            rospy.sleep(1.0)
            self.open_stopper()
            rospy.sleep(1.0)
            ###### ⭐ 여기까지 ######

            self.state = 101
            rospy.loginfo("✅ Finished second orange → state101")

        # 🔴 빨간색만 감지 → state101로 복귀
        elif has_red2 and not has_orange2:
            rospy.loginfo("🔴 Red only detected → state101")

            ###### ⭐ 여기 추가 ######
            rospy.sleep(1.0)
            self.open_stopper()
            rospy.sleep(1.0)
            ###### ⭐ 여기까지 ######

            self.state = 101

        # 🚫 아무 감지도 없음 → hold
        else:
            rospy.loginfo("⏸ state202 hold (no orange/red detected)")


    # ============================================================
    # 🩶 state_301: 첫 번째 회색 원 감지 및 픽업 → grid에 놓기 → state_302 전환
    # ============================================================
    def state_301(self):
        rospy.loginfo("➡️ Enter state_301()")

        # ① 시작 자세
        self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS")
        rospy.sleep(3.0)

        # ② 회색 원 감지 시도 (/grid1_fallback/result)
        rospy.loginfo("🕐 Waiting for /grid1_fallback/result ...")
        fb_msg = None
        recent_points = []
        t0 = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - t0) < 5.0 and not rospy.is_shutdown():
            msg = self.grid1_fallback.get()
            if not msg or (msg.x == 0.0 and msg.y == 0.0):
                rospy.sleep(0.05)
                continue

            # 🚧 Outlier filter
            if abs(msg.x) > 200 or abs(msg.y) > 200:
                rospy.logwarn(f"⚠️ Ignored abnormal fallback detection: x={msg.x:.1f}, y={msg.y:.1f}")
                continue

            # 🚧 Stability filter
            recent_points.append((msg.x, msg.y))
            if len(recent_points) > 15:
                recent_points.pop(0)

            if len(recent_points) >= 4:
                xs, ys = zip(*recent_points)
                std_x, std_y = np.std(xs), np.std(ys)
                if std_x < 3.0 and std_y < 3.0:
                    fb_msg = msg
                    rospy.loginfo("✅ Stable fallback detection confirmed.")
                    break
            rospy.sleep(0.05)

        has_red = self.point_detected(self.red_result)
        has_gray_circle = fb_msg is not None

        # ------------------------------------------------------------
        # 🩶 회색 원 감지 성공 → 픽업 절차 수행
        # ------------------------------------------------------------
        if has_gray_circle:
            dx = fb_msg.x / 1000.0
            dy = fb_msg.y / 1000.0
            rospy.loginfo(f"📍 Gray circle offset: dx={dx:.3f} m, dy={dy:.3f} m")
            rospy.sleep(2.0)

            # XY 이동
            rospy.loginfo("➡️ Moving in XY plane first...")
            self.ur5e.go_to_pose_rel([dx, dy, 0.0], [0.0, 0.0, 0.0])
            rospy.sleep(0.8)

            # 하강 → Grip ON → 상승
            self.ur5e.go_to_pose_rel([0, 0, -0.07], [0, 0, 0])
            self.grip_on_6s()
            self.ur5e.go_to_pose_rel([0, 0, 0.115], [0, 0, 0])

            # 관찰자세 이동 후 그리드로 이동
            self.goJ(self.OBS_GRID_JOINTS, "OBS_GRID_JOINTS")
            msg = self.inv_gray_result.get()
            cell = self.choose_cell_from_string(msg.data if msg else "")

            if cell and cell in self._FALLBACK_POSES:
                start_pose = self._FALLBACK_POSES[cell]["start"]
                pick_pose = self._FALLBACK_POSES[cell]["pick"]
                self.goJ(start_pose, f"{cell} start (gray)")
                rospy.sleep(0.5)
                self.goJ(pick_pose, f"{cell} pick (gray)")
                rospy.loginfo("✋ Grip OFF (6s)")
                self.grip_off_6s()
                rospy.sleep(0.5)
                self.goJ(start_pose, f"{cell} return start (gray)")
            else:
                rospy.logwarn("❌ No valid inv-gray grid detected.")

            # ✅ 다음 단계로 전환
            self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")
            self.state = 302
            rospy.loginfo("➡️ Move to state_302")

        # 🔴 빨간 원만 감지 → 픽 없이 바로 state_302 전환
        elif has_red and not has_gray_circle:
            rospy.loginfo("🔴 Red detected only → move to ASM_RED_DETECT_TRIGGER_JOINTS")
            self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")
            self.state = 302
            rospy.loginfo("➡️ state_302")

        # 🚫 아무 감지도 없음
        else:
            rospy.loginfo("⏸ state301 hold (no detection)")


    # ============================================================
    # 🩶 state_302: 두 번째 회색 원 감지 및 픽업 → grid에 놓기 → state_1 복귀
    # ============================================================
    def state_302(self):
        rospy.loginfo("➡️ Enter state_302()")

        # ① 시작 포즈 (레드 트리거 위치)
        self.goJ(self.ASM_RED_DETECT_TRIGGER_JOINTS, "ASM_RED_DETECT_TRIGGER_JOINTS")

        # ② 감지 시도
        rospy.loginfo("🕐 Waiting for /grid1_fallback/result ...")
        fb_msg = None
        recent_points = []
        t0 = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - t0) < 5.0 and not rospy.is_shutdown():
            msg = self.grid1_fallback.get()
            if not msg or (msg.x == 0.0 and msg.y == 0.0):
                rospy.sleep(0.05)
                continue

            if abs(msg.x) > 200 or abs(msg.y) > 200:
                rospy.logwarn(f"⚠️ Ignored abnormal fallback detection: x={msg.x:.1f}, y={msg.y:.1f}")
                continue

            recent_points.append((msg.x, msg.y))
            if len(recent_points) > 15:
                recent_points.pop(0)

            if len(recent_points) >= 4:
                xs, ys = zip(*recent_points)
                std_x, std_y = np.std(xs), np.std(ys)
                if std_x < 3.0 and std_y < 3.0:
                    fb_msg = msg
                    rospy.loginfo("✅ Stable fallback detection confirmed.")
                    break
            rospy.sleep(0.05)

        has_red = self.point_detected(self.red_result)
        has_gray_circle = fb_msg is not None

        # 🩶 회색 원 감지 성공 시 → 픽업 후 grid 놓기
        if has_gray_circle:
            dx = fb_msg.x / 1000.0
            dy = fb_msg.y / 1000.0
            rospy.loginfo(f"📍 Gray circle offset: dx={dx:.3f} m, dy={dy:.3f} m")
            rospy.sleep(2.0)
            
            self.ur5e.go_to_pose_rel([dx, dy, 0], [0, 0, 0])
            rospy.sleep(0.8)
            self.ur5e.go_to_pose_rel([0, 0, -0.07], [0, 0, 0])
            self.grip_on_6s()
            self.ur5e.go_to_pose_rel([0, 0, 0.115], [0, 0, 0])

            # 관찰자세로 복귀 후 grid 놓기
            self.goJ(self.OBS_GRID_JOINTS, "OBS_GRID_JOINTS")
            msg = self.inv_gray_result.get()
            cell = self.choose_cell_from_string(msg.data if msg else "")

            if cell and cell in self._FALLBACK_POSES:
                start_pose = self._FALLBACK_POSES[cell]["start"]
                pick_pose = self._FALLBACK_POSES[cell]["pick"]
                self.goJ(start_pose, f"{cell} start (gray2)")
                rospy.sleep(0.5)
                self.goJ(pick_pose, f"{cell} pick (gray2)")
                rospy.loginfo("✋ Grip OFF (6s)")
                self.grip_off_6s()
                rospy.sleep(0.5)
                self.goJ(start_pose, f"{cell} return start (gray2)")
            else:
                rospy.logwarn("❌ No valid inv-gray grid detected (state302).")

            # ✅ 완료 후 state 1 복귀
            self.goJ(self.ASM_OBS_START_JOINTS, "ASM_OBS_START_JOINTS")

            ###### ⭐ 여기 추가 ######
            rospy.sleep(1.0)
            self.open_stopper()
            rospy.sleep(1.0)
            ###### ⭐ 여기까지 ######

            self.state = 1
            rospy.loginfo("✅ Finished second gray → state 1")

        # 🔴 빨간 원만 감지 → pick 없이 바로 state 1 전환
        elif has_red and not has_gray_circle:
            rospy.loginfo("🔴 Red only detected → skip pick → state 1")

            ###### ⭐ 여기 추가 ######
            rospy.sleep(1.0)
            self.open_stopper()
            rospy.sleep(1.0)
            ###### ⭐ 여기까지 ######

            self.state = 1

        # 🚫 아무 감지도 없음 → hold
        else:
            rospy.loginfo("⏸ state302 hold (no detection)")



if __name__ == "__main__":
    node = AsmFsmNode()
    node.spin()
