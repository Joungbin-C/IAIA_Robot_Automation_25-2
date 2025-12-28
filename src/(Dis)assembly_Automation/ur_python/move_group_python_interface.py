#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs

import tf
import copy
import math

from tf.transformations import *
from geometry_msgs.msg import Quaternion
from moveit_commander.conversions import pose_to_list
from math import pi, tau, dist, fabs, cos

from ur_msgs.srv import SetIO, SetIORequest
from ur_python.msg import robot_state


def all_close(goal, actual, tolerance):
    """Compare poses or lists within a tolerance."""
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        d = dist((x1, y1, z1), (x0, y0, z0))
        cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def ensure_minimal_rotation(self, current_quat, target_quat):
        """Quaternion 부호 반전으로 인한 180° jump 방지"""
        dot = sum(c * t for c, t in zip(current_quat, target_quat))
        if dot < 0.0:
            target_quat = [-t for t in target_quat]
        return target_quat

    def __init__(self, real="real", gripper="gripper"):
        super(MoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        self.msg_robot_state = robot_state()
        self.pub_robot_state = rospy.Publisher("robot_state", robot_state, queue_size=10)

        self.robot = moveit_commander.RobotCommander()
        self.group_name = "manipulator"
        self.manipulator = moveit_commander.move_group.MoveGroupCommander(self.group_name)

        if real == "real":
            self.io_handler = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
            self.gripper = SetIORequest()
            self.gripper_init()

        self.planning_frame = self.manipulator.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)
        self.eef_link = self.manipulator.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
        rospy.sleep(1)

    def move_to_standby(self):
        print("============ Go to 'stand_by' pose")
        self.go_to_joint_abs([tau/4, -tau/4, tau/4, -tau/4, -tau/4, 0.0])
        self.manipulator.go(wait=True)
        self.manipulator.stop()
        print("============ Printing robot state")
        print(self.robot.get_current_state())

    def gripper_init(self):
        self.gripper.fun = 1
        self.gripper.pin = 1
        self.grip_off()

    def grip_on(self):
        self.msg_robot_state.move = 1
        self.pub_robot_state.publish(self.msg_robot_state)
        self.gripper.state = 1
        self.io_handler.call(self.gripper)
        rospy.sleep(1.5)
        self.msg_robot_state.move = 0
        self.pub_robot_state.publish(self.msg_robot_state)

    def grip_off(self):
        self.msg_robot_state.move = 1
        self.pub_robot_state.publish(self.msg_robot_state)
        self.gripper.state = 0
        self.io_handler.call(self.gripper)
        rospy.sleep(1.5)
        self.msg_robot_state.move = 0
        self.pub_robot_state.publish(self.msg_robot_state)

    def go_to_joint_abs(self, target_joints):
        current_joint = self.manipulator.get_current_joint_values()
        target_joint = copy.deepcopy(current_joint)
        for i in range(6):
            target_joint[i] = target_joints[i]

        self.msg_robot_state.move = 1
        self.pub_robot_state.publish(self.msg_robot_state)
        self.manipulator.go(target_joint, wait=True)
        self.manipulator.stop()
        self.msg_robot_state.move = 0
        self.pub_robot_state.publish(self.msg_robot_state)
        return all_close(target_joint, self.manipulator.get_current_joint_values(), 0.005)

    def go_to_joint_rel(self, relative_pos):
        current_joint = self.manipulator.get_current_joint_values()
        target_joint = [cj + r for cj, r in zip(current_joint, relative_pos)]
        self.msg_robot_state.move = 1
        self.pub_robot_state.publish(self.msg_robot_state)
        self.manipulator.go(target_joint, wait=True)
        self.manipulator.stop()
        self.msg_robot_state.move = 0
        self.pub_robot_state.publish(self.msg_robot_state)
        return all_close(target_joint, self.manipulator.get_current_joint_values(), 0.005)

    def go_to_pose_abs(self, absolute_xyz, absolute_rpy):
        current_pose = self.manipulator.get_current_pose().pose
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.x = absolute_xyz[0]
        target_pose.position.y = absolute_xyz[1]
        target_pose.position.z = absolute_xyz[2]
        target_quat = quaternion_from_euler(*absolute_rpy)
        target_pose.orientation = Quaternion(*target_quat)
        self.manipulator.set_pose_target(target_pose)
        self.msg_robot_state.move = 1
        self.pub_robot_state.publish(self.msg_robot_state)
        self.manipulator.go(wait=True)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()
        self.msg_robot_state.move = 0
        self.pub_robot_state.publish(self.msg_robot_state)
        return all_close(target_pose, self.manipulator.get_current_pose().pose, 0.005)

    def go_to_pose_rel(self, relative_xyz, relative_rpy, max_retry=3):
        """
        Move the manipulator to a relative pose (position + orientation)
        with retry and stability.
        relative_xyz: [dx, dy, dz] in meters
        relative_rpy: [dR, dP, dY] in radians
        """

        # ① 현재 pose 읽기
        current_pose = self.manipulator.get_current_pose().pose
        target_pose = copy.deepcopy(current_pose)

        # ② 상대 이동 적용
        target_pose.position.x += relative_xyz[0]
        target_pose.position.y += relative_xyz[1]
        target_pose.position.z += relative_xyz[2]

        # ③ 회전 (쿼터니언 누적 방식)
        current_quat = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ]
        rel_quat = quaternion_from_euler(*relative_rpy)

        # 누적 회전 (상대 회전 * 현재 회전)
        target_quat = quaternion_multiply(rel_quat, current_quat)

        # minimal rotation 보정
        dot = sum(c * t for c, t in zip(current_quat, target_quat))
        if dot < 0.0:
            target_quat = [-q for q in target_quat]
            rospy.logwarn("🌀 Quaternion flipped for minimal rotation")

        target_pose.orientation = Quaternion(*target_quat)

        # 로그 출력
        rospy.loginfo(f"🎯 Δxyz = {relative_xyz}, Δrpy(deg) = {[round(x*180/pi,3) for x in relative_rpy]}")
        rospy.loginfo(f"📍 From z={current_pose.position.z:.3f} → To z={target_pose.position.z:.3f}")

        # ④ Cartesian Path 기반 경로 계획 및 실행 (재시도)
        waypoints = [target_pose]
        success = False

        for attempt in range(1, max_retry + 1):
            (plan, fraction) = self.manipulator.compute_cartesian_path(
                waypoints,
                0.002,   # eef_step (단위: m)
                True     # jump_threshold
            )
            rospy.loginfo(f"🧭 Attempt {attempt}: Cartesian path fraction={fraction:.3f}")

            if fraction > 0.9:
                rospy.loginfo(f"✅ Cartesian path success on attempt {attempt}")
                self.msg_robot_state.move = 1
                self.pub_robot_state.publish(self.msg_robot_state)

                self.manipulator.execute(plan, wait=True)
                self.manipulator.stop()

                self.msg_robot_state.move = 0
                self.pub_robot_state.publish(self.msg_robot_state)
                success = True
                break
            else:
                rospy.logwarn(f"⚠️ Path incomplete (fraction={fraction:.3f}), retrying...")
                rospy.sleep(0.5)

        # ⑤ 실패 시 안전 중단
        if not success:
            rospy.logerr("❌ Cartesian path failed after retries — skipping safely.")
            return False

        # ⑥ 최종 자세 비교
        current_pose = self.manipulator.get_current_pose().pose
        if not all_close(target_pose, current_pose, 0.003):
            rospy.logwarn("⚠️ Final pose deviation detected, check accuracy.")

        return True
