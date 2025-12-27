#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, time
from math import radians, tau
from indy_driver.msg import comp_wind_info, decomp_wind_info, bracket_info, comp_pose_info, cell_state_info, process_info, comp_pose_info, decomp_pose_info
from move_group_python_interface import MoveGroupPythonInterface
import serial
import math  

CELL_JOINTS_ABS = {
    1: [radians(x) for x in [17.07, 30.93, 16.57, -0.03, 131.63, 17.10]],
    2: [radians(x) for x in [11.63, 26.03, 26.64, -0.04, 127.02, 11.78]],   
    3: [radians(x) for x in [ 5.37, 25.02, 26.48, -0.00, 128.23,  5.71]],
    4: [radians(x) for x in [19.07, 20.99, 30.35, -0.09, 127.19, 19.25]],
    5: [radians(x) for x in [13.29, 19.97, 31.78,  0.00, 128.77, 13.87]],
    6: [radians(x) for x in [ 6.08, 16.91, 35.98, -0.00, 127.54,  6.10]],
}

CELL_JOINTS_ABS2 = {
    1: [radians(x) for x in [16.72, 21.16, 43.88,  0.39, 114.90, 17.07]],
    2: [radians(x) for x in [11.30, 19.38, 46.29,  0.39, 115.25, 11.73]],   
    3: [radians(x) for x in [ 5.37, 16.04, 50.65, -0.00, 113.16,  5.71]],
    4: [radians(x) for x in [19.08, 11.63, 56.12, -0.09, 110.74, 19.27]],
    5: [radians(x) for x in [13.29, 10.88, 57.02,  0.00, 112.63, 13.87]],
    6: [radians(x) for x in [ 6.08,  8.52, 59.77, -0.00, 112.12,  6.20]],
}

CELL_JOINTS_ABS3 = {
    1: [radians(x) for x in [19.67,  9.80, 53.00, -0.00, 117.16, 19.80]],
    2: [radians(x) for x in [13.43,  7.42, 55.75,  0.00, 116.82, 13.46]],
    3: [radians(x) for x in [ 6.47,  4.28, 59.21,  0.05, 116.49,  6.57]],
    4: [radians(x) for x in [22.67,  2.03, 61.60, -0.01, 116.34, 22.75]],
    5: [radians(x) for x in [15.69, -0.06, 63.89, -0.21, 116.23, 15.54]],
    6: [radians(x) for x in [ 7.65, -2.15, 65.78, -0.24, 116.40,  7.53]],
}

check_pose          = [radians(x) for x in [ 13.65,  16.27 , 34.63,  0.00, 129.09, 13.64]]               # checking windings
comp_pose           = [radians(x) for x in [ 56.29,  -7.47 , 61.91,  0.00, 125.55, 56.31]]               # checking braket is arriver
comp_pose_place1    = [radians(x) for x in [ 46.36,  -3.20 , 62.98,  0.00, 120.70, 46.04]]               # bracket num 1
comp_pose_place2    = [radians(x) for x in [ 27.50, -13.75 , 71.86,  0.13, 122.30, 27.02]]               # bracket num 2
decomp_pose_place1  = [radians(x) for x in [ 58.23, -11.81 , 73.84, -0.00, 118.35, 57.75]]               # checking braket num 1 in decomp mode
decomp_pose_place2  = [radians(x) for x in [ 39.24, -25.30 , 82.55, -0.13, 122.75, 38.73]]               # checking braket num 2 in decomp mode
trash_pose          = [radians(x) for x in [100.02,  39.10 , 20.85,  0.01, 118.76, 46.65]]               # trash position
b_arrive_check_pose = [radians(x) for x in [ 58.66,  -6.64 , 60.56,  0.32, 127.38, 59.84]]               # checking braket is arrive


class IndyPickPlaceNode:
    def __init__(self):
        self.robot = MoveGroupPythonInterface(real=True, gripper="Gripper")
        self.latest_comp_wind   = None                                          
        self.latest_decomp_wind = None
        self.latest_bracket     = None
        self.latest_flag        = None  # 1~4
        self.latest_decomp_pose = None
        self.latest_comp_pose   = None
        self.process_flag       = None  # compose or decompose
        self.iteration_count    = 0
        self.MAX_ITER           = 6  
        self.first              = None
        self.second             = None

        # --- Subscribers --- #
        rospy.Subscriber("comp_wind_info"  , comp_wind_info  , self.comp_wind_callback  )             # image_Display1
        rospy.Subscriber("decomp_wind_info", decomp_wind_info, self.decomp_wind_callback)             # image_Display6
        rospy.Subscriber("bracket_info"    , bracket_info    , self.bracket_callback    )             # image_Display2    = Bracket 도착여부
        rospy.Subscriber("comp_pose_info"  , comp_pose_info  , self.comp_pose_callback  )             # image_Display3
        rospy.Subscriber("cell_state_info" , cell_state_info , self.cell_state_callback )             # Mid_communication
        rospy.Subscriber("process_info"    , process_info    , self.process_callback    )             # image_Display4    = Process 결정여부
        rospy.Subscriber("decomp_pose_info", decomp_pose_info, self.decomp_pose_callback)             # image_Display5


        # --- Arduino Serial (모터 제어) --- #
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # 포트는 환경에 맞게 변경
            rospy.sleep(2.0)  # 아두이노 자동 리셋 대기
            rospy.loginfo("🔌 Arduino serial connected.")
        except Exception as e:
            self.ser = None
            rospy.logwarn(f"⚠️ Arduino serial open failed: {e}")

        rospy.loginfo("🤖 [indy_pick_place] Initialized and waiting for cell/place info...")

    # ========================================================================= #  
    # =========================== Callback Function =========================== #
    def comp_wind_callback(self, msg):                                                       
        self.latest_comp_wind = int(msg.cell_num)

    def decomp_wind_callback(self, msg):
        self.latest_decomp_wind = int(msg.cell_num)

    def bracket_callback(self, msg):
        self.latest_bracket = (msg.center_x, msg.center_y)

    def cell_state_callback(self, msg: cell_state_info):
        self.latest_flag = int(msg.flag)  
        rospy.loginfo(f"[cell_state] Updated flag={self.latest_flag}")


    def process_callback(self, msg):
        self.process_flag = int(msg.flag) 

    def comp_pose_callback(self, msg):
        self.latest_comp_pose = (msg.center_x, msg.center_y)

    def decomp_pose_callback(self, msg):
        self.latest_decomp_pose = (msg.center_x, msg.center_y)

    # ========================================================================= #  
    # ============================= Wait Function ============================= #
    def wait_until_wind_ready(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            c = self.latest_comp_wind
            if c is not None :  
                return c
            rospy.loginfo_throttle(1.0, "🕓 Waiting for valid cell ...")
            rate.sleep()


    def wait_until_bracket_ready(self):
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            p = self.latest_bracket
            if p is not None:
                x, y = p
                if math.isfinite(x) and math.isfinite(y):
                    return (x, y)
            rospy.loginfo_throttle(1.0, "🕓 Waiting for valid bracket (not NaN)...")

    def wait_until_process_ready(self):
        rate = rospy.Rate(50)  # 50 Hz polling
        while not rospy.is_shutdown():
            f = getattr(self, "process_flag", None)  # 혹시 변수 미정의 상태 대비
            if f in (0, 1):  
                rospy.loginfo(f"✅ process_flag received: {f} ({'compine' if f == 1 else 'decompose'})")
                return f
            rospy.loginfo_throttle(1.0, "🕓 Waiting for process_flag (0=decompose, 1=compine)...")
            rate.sleep()

    def wait_until_decomp_pose_ready(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            p = self.latest_decomp_pose
            if p is not None:
                cx, cy = p
                if math.isfinite(cx) and math.isfinite(cy):
                    rospy.loginfo(f"✅ decompose pose ready: ({cx:.4f}, {cy:.4f})")
                    return (cx, cy)
            rospy.loginfo_throttle(1.0, "🕓 Waiting for valid decompose pose (not None/NaN)...")


    # ============================ Arduino Function =========================== #
    # ========================================================================= #  

    def open_stopper(self):
        """아두이노에 'o' 문자 전송 → 모터 오픈"""
        if not hasattr(self, 'ser') or self.ser is None:
            rospy.logwarn("⚠️ Arduino serial not available.")
            return
        try:
            self.ser.write(b'o')
            self.ser.flush()
            rospy.loginfo("📨 Sent 'o' to Arduino (open stopper).")
        except Exception as e:
            rospy.logwarn(f"⚠️ Failed to send 'o' to Arduino: {e}")



    def compose_process(self, cell):
        rospy.loginfo(f"🚀 Starting iteration {self.iteration_count+1}/{self.MAX_ITER} )")
        start = time.time()

        flag  = self.latest_flag
        rospy.loginfo(f"{flag}")
        if flag is None :
            flag = 1

        if flag == 1:
            self.first  = 1
            self.second = 1
        elif flag == 2:
            self.first  = 3
            self.second = 1
        elif flag == 3:
            self.first  = 1
            self.second = 3
        elif flag == 4:
            self.first  = 3
            self.second = 3
        elif flag == 5:
            self.first  = 1
            self.second = 2
        elif flag == 6:
            self.first  = 3
            self.second = 2
        else :
            self.first  = 1
            self.second = 1

        

        try:
            if self.first == 1 :
                #====================================================== PICK ======================================================================#
                self.robot.go_to_joint_abs(check_pose)
                rospy.sleep(0.5)

                if self.latest_comp_wind is None:
                    self.wait_until_wind_ready()
                
                if self.latest_comp_wind in (0,6):
                    rospy.loginfo (("⚠️ End compose process"))
                    return
                    
                wind = self.latest_comp_wind
                rospy.loginfo(f"📦 Updated cell from callback: {wind}")

                target_joints = CELL_JOINTS_ABS3.get(wind)
                self.robot.go_to_joint_abs(target_joints)
                rospy.sleep(0.5)

                cx, cy = self.latest_comp_pose
            
                self.robot.go_to_pose_rel([-cy, cx, 0.0],[0,0,0])
                rospy.sleep(0.5)

                self.robot.go_to_pose_rel([0.0,0.0, -0.04],[0,0,0])
                rospy.sleep(0.5)

                self.robot.grip_on()
                rospy.sleep(1)

                self.robot.go_to_joint_rel([0, radians(-8.0), 0, 0, radians(-5.0), 0])
                rospy.sleep(0.5)
            
                #====================================================== PLACE =====================================================================#
                self.robot.go_to_joint_abs(comp_pose_place1)
                rospy.sleep(0.5)

                self.robot.grip_off()
                rospy.sleep(1)

            if self.second == 1:
                #=================================================================================================================================#
                #====================================================== PICK ======================================================================#
                self.robot.go_to_joint_abs(check_pose)
                rospy.sleep(0.5)

                if self.latest_comp_wind is None:
                    self.wait_until_wind_ready()
                
                # if self.latest_comp_wind == 6:
                #     rospy.loginfo (("⚠️ There is no cell: End compose process"))
                #     return
                wind = self.latest_comp_wind
                rospy.loginfo(f"📦 Updated cell from callback: {wind}")

                target_joints = CELL_JOINTS_ABS3.get(wind)
                self.robot.go_to_joint_abs(target_joints)
                rospy.sleep(0.5)

                cx, cy = self.latest_comp_pose
                self.robot.go_to_pose_rel([-cy, cx, 0.0],[0,0,0])
                rospy.sleep(0.5)

                self.robot.go_to_pose_rel([0.0,0.0, -0.04],[0,0,0])
                rospy.sleep(0.5)

                self.robot.grip_on()
                rospy.sleep(1)

                self.robot.go_to_joint_rel([0, radians(-8.0), 0, 0, radians(-5.0), 0])
            
                #====================================================== PLACE =====================================================================#
                self.robot.go_to_joint_abs(comp_pose_place2)
                rospy.sleep(0.5)

                self.robot.grip_off()
                rospy.sleep(1)
            
            
            if self.first == 3 :
                self.robot.go_to_joint_abs(comp_pose_place1)
                rospy.sleep(0.5)
                self.robot.go_to_pose_rel([0,0,-0.13],[0,0,0])
                rospy.sleep(0.5)
                self.robot.grip_on()
                rospy.sleep(1)
                self.robot.go_to_pose_rel([0,0,0.1],[0,0,0])
                rospy.sleep(0.5)
                self.robot.go_to_joint_abs(trash_pose)
                rospy.sleep(0.5)
                self.robot.grip_off()
                rospy.sleep(1)


            if self.second == 3 :
                self.robot.go_to_joint_abs(comp_pose_place2)
                rospy.sleep(0.5)
                self.robot.go_to_pose_rel([0,0,-0.13],[0,0,0])
                rospy.sleep(0.5)
                self.robot.grip_on()
                rospy.sleep(1)
                self.robot.go_to_pose_rel([0, 0, 0.1],[0,0,0])
                rospy.sleep(0.5)
                self.robot.go_to_joint_abs(trash_pose)
                rospy.sleep(0.5)
                self.robot.grip_off()
                rospy.sleep(1)
                




        except Exception as e:
            rospy.logerr(f"⚠️ [indy_pick_place] ERROR during iteration: {e}")
        finally:
            rospy.loginfo(f"✅ Iteration finished in {time.time() - start:.2f}s")



    def decompose_process(self):
        rospy.loginfo(f"🚀 Starting decomposition process {self.iteration_count+1}/{self.MAX_ITER})")
        start = time.time()

        try:
            #====================================================== PICK =====================================================================#
            self.robot.go_to_joint_abs(decomp_pose_place1)
            rospy.sleep(1)
            cx, cy = self.wait_until_decomp_pose_ready()

            rospy.loginfo(f"📍 Using decompose pose: cx={cx:.4f}, cy={cy:.4f}")

            self.robot.go_to_pose_rel([-cy, cx, -0.015],[0,0,0])
            rospy.sleep(0.5)
            self.robot.go_to_pose_rel([  0,  0, -0.010],[0,0,0])
            rospy.sleep(0.5)
            self.robot.grip_on()
            rospy.sleep(1)
            self.robot.go_to_pose_rel([  0,  0, 0.100],[0,0,0])
            rospy.sleep(0.5)

            #====================================================== PLACE =====================================================================#
            self.robot.go_to_joint_abs(check_pose)
            rospy.sleep(0.5)

            wind = self.latest_decomp_wind
            rospy.loginfo(f"📦 Updated wind from callback: {wind}")

            target_joints = CELL_JOINTS_ABS.get(wind)
            self.robot.go_to_joint_abs(target_joints)
            rospy.sleep(0.5)

            target_joints = CELL_JOINTS_ABS2.get(wind)
            self.robot.go_to_joint_abs(target_joints)
            rospy.sleep(0.5)

            self.robot.grip_off()
            rospy.sleep(1)

            self.robot.go_to_joint_rel([0, radians(-8.0), 0, 0, radians(-5.0), 0])
            rospy.sleep(0.5)

            #=================================================================================================================================#
            #====================================================== PICK =====================================================================#

            self.robot.go_to_joint_abs(decomp_pose_place2)
            rospy.sleep(1.0)

            cx, cy = self.latest_decomp_pose
            if not (math.isfinite(cx) and math.isfinite(cy)):
                self.open_stopper()
                return

            rospy.loginfo(f"📍 Using decompose pose: cx={cx:.4f}, cy={cy:.4f}")

            self.robot.go_to_pose_rel([-cy, cx, -0.015],[0,0,0])
            rospy.sleep(0.5)
            self.robot.go_to_pose_rel([  0,  0, -0.010],[0,0,0])
            rospy.sleep(0.5)
            self.robot.grip_on()
            rospy.sleep(1)
            self.robot.go_to_pose_rel([  0,  0, 0.100],[0,0,0])
            rospy.sleep(0.5)


            #====================================================== PLACE =====================================================================#
            self.robot.go_to_joint_abs(check_pose)
            rospy.sleep(0.5)

            wind = self.latest_decomp_wind
            rospy.loginfo(f"📦 Updated wind from callback: {wind}")

            target_joints = CELL_JOINTS_ABS.get(wind)
            self.robot.go_to_joint_abs(target_joints)
            rospy.sleep(0.5)

            target_joints = CELL_JOINTS_ABS2.get(wind)
            self.robot.go_to_joint_abs(target_joints)
            rospy.sleep(0.5)

            self.robot.grip_off()
            rospy.sleep(1)

            self.robot.go_to_joint_rel([0, radians(-8.0), 0, 0, radians(-5.0), 0])
            rospy.sleep(0.5)




        except Exception as e:
            rospy.logerr(f"⚠️ [indy_pick_place] ERROR during iteration: {e}")
        finally:
            rospy.loginfo(f"✅ Iteration finished in {time.time() - start:.2f}s")


    def run(self):
        
        self.open_stopper()
        rospy.sleep(15)
        # ====================================================================================================== #
        self.robot.go_to_joint_abs(b_arrive_check_pose)
        rospy.sleep(0.5)

        if (self.latest_bracket is None 
            or not all(math.isfinite(v) for v in self.latest_bracket)):
            self.wait_until_bracket_ready()
        
        rospy.sleep(5)

        rospy.loginfo(f"📦 Bracket is arrive")


        # ② 셀 감지 여부로 mode 판정 ============================================================================== #

        if self.process_flag not in (0, 1):
            self.wait_until_process_ready() 

        if self.process_flag == 1:
            rospy.loginfo("✅ Cell detected → compine mode selected.")
            self.compine_mode = True
        else:
            rospy.loginfo("❌ No cell detected → decompose mode selected.")
            self.compine_mode = False


        if self.compine_mode:
            self.compose_process(self.latest_comp_wind)
        else:
            self.decompose_process()


if __name__ == "__main__":
    node = IndyPickPlaceNode()
    node.run()
