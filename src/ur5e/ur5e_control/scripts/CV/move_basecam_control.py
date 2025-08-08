#!/usr/bin/env python3

import sys
import rospy
import tf
import numpy as np
import copy
from geometry_msgs.msg import Pose, Point, Quaternion
from ur5e_control.msg import ObjectPose, MultiObjectPose 
import moveit_commander
from tf.transformations import quaternion_matrix, quaternion_from_matrix, quaternion_about_axis, quaternion_multiply
from math import radians

class BaseCameraBasedPoseTransformerPlanner:
    def __init__(self):

        rospy.loginfo("PoseTransformerPlanner node initialized.")
        self.request_flag = False

    def get_end_effector_pose(self):
        listener = tf.TransformListener()
        base_frame = "panda_link0"
        ee_frame = "panda_hand_tcp"
        # 等待 TF 可用
        listener.waitForTransform(base_frame, ee_frame, rospy.Time(0), rospy.Duration(4.0))

        try:
            current_pose = Pose()
            (trans, rot) = listener.lookupTransform(base_frame, ee_frame, rospy.Time(0))
            print("Position: ", trans)
            print("Orientation: ", rot)
            current_pose.position = Point(trans[0], trans[1], trans[2])
            current_pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
            return current_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to get transform from {} to {}".format(base_frame, ee_frame))
            return None
    
    def target_pose_compute(self, object: str):
        ###########################################################################
        ### Server-Client
        ###########################################################################
        # response = self.client.call(True)
        # Q_obj2gripper = response.target_pose
        ###########################################################################
        ### Publisher-Subscriber
        ###########################################################################
        A_Q_obj2robot = rospy.wait_for_message('/base_cam_target_pose', MultiObjectPose)
        Q_obj2robot = Pose()
        for obj in A_Q_obj2robot.objects:
            if obj.name == object:
                Q_obj2robot = copy.deepcopy(obj.pose)
                rospy.loginfo(f"Found {object} pose:")
                rospy.loginfo("Position: x=%.3f y=%.3f z=%.3f", obj.pose.position.x, obj.pose.position.y, obj.pose.position.z)
                rospy.loginfo("Orientation: x=%.3f y=%.3f z=%.3f w=%.3f",
                            obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w)
                break
    
        rospy.loginfo("Get Object Pose related to TCP - x: %.4f, y: %.4f, z: %.4f", Q_obj2robot.position.x, Q_obj2robot.position.y, Q_obj2robot.position.z)
        rospy.loginfo("Received external pose.")

        # 获取当前末端执行器姿态
        current_pose = self.get_end_effector_pose()
        rospy.loginfo("Current Position - (x: %.4f, y: %.4f, z: %.4f)", current_pose.position.x, current_pose.position.y, current_pose.position.z)
        rospy.loginfo("Current Orientation - (x: %.4f, y: %.4f, z: %.4f, w: %.4f)", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
        
        # T_target = RotationFitter.fit_to_flipX_then_rotateZ(T_target)
        # 转换为 geometry_msgs/Pose
        target_pose = copy.deepcopy(current_pose)

        # Since the robot is not mounted upright but tilted by 15 degrees, a rotation is added to link8 to maintain the TCP in a horizontal orientation.”（z -15°）
        rotation_quat = quaternion_about_axis(radians(-15), (0, 0, 1))
        original_orientation = [1, 0, 0, 0,]
        new_orientation = quaternion_multiply(rotation_quat, original_orientation)

        target_pose.position.x = Q_obj2robot.position.x - 0.05
        target_pose.position.y = Q_obj2robot.position.y
        target_pose.position.z = 0.15
        target_pose.orientation.x = new_orientation[0]
        target_pose.orientation.y = new_orientation[1]
        target_pose.orientation.z = new_orientation[2]
        target_pose.orientation.w = new_orientation[3]

        rospy.loginfo("Target Position - (x: %.4f, y: %.4f, z: %.4f)", target_pose.position.x, target_pose.position.y, target_pose.position.z)
        rospy.loginfo("Target Orientation - (x: %.4f, y: %.4f, z: %.4f, w: %.4f)", target_pose.orientation.x, target_pose.orientation.y, Q_obj2robot.orientation.z, Q_obj2robot.orientation.w)

        return target_pose
        # jump_threshold = 0.0
        # eef_step = 0.01
        # (plan, _) = self.move_group.compute_cartesian_path([target_pose], eef_step, jump_threshold)

        # # 执行轨迹
        # plan.joint_trajectory
        # self.move_group.execute(plan, wait=True)

    def shutdown(self):
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    planner = BaseCameraBasedPoseTransformerPlanner()
    try:
        planner.target_pose_compute("chair_nut2")
    except rospy.ROSInterruptException:
        planner.shutdown()
