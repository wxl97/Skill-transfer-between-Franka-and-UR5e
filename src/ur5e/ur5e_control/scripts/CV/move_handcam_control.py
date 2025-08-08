#!/usr/bin/env python3

import sys
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from ur5e_control.msg import ObjectPose
import moveit_commander
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import copy

class HandCameraBasedPoseTransformerPlanner:
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
    

    def target_pose_compute(self):
        
        Q_obj2gripper = rospy.wait_for_message('/target_pose', Pose)
        
        rospy.loginfo("Get Object Pose related to TCP - x: %.4f, y: %.4f, z: %.4f", Q_obj2gripper.position.x, Q_obj2gripper.position.y, Q_obj2gripper.position.z)
        rospy.loginfo("Received external pose.")
        T_obj2gripper = self.pose_to_matrix(Q_obj2gripper)

        # 获取当前末端执行器姿态
        current_pose = self.get_end_effector_pose()
        rospy.loginfo("Current Position - (x: %.4f, y: %.4f, z: %.4f)", current_pose.position.x, current_pose.position.y, current_pose.position.z)
        rospy.loginfo("Current Orientation - (x: %.4f, y: %.4f, z: %.4f, w: %.4f)", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
        T_gripper2robot = self.pose_to_matrix(current_pose)

        # 计算目标变换矩阵
        T_target = np.dot(T_gripper2robot, T_obj2gripper)
        # T_target = RotationFitter.fit_to_flipX_then_rotateZ(T_target)
        # 转换为 geometry_msgs/Pose
        target_pose = copy.deepcopy(current_pose)
        Q_obj2robot = self.matrix_to_pose(T_target)
        rospy.loginfo("Target Position - (x: %.4f, y: %.4f, z: %.4f)", Q_obj2robot.position.x, Q_obj2robot.position.y, Q_obj2robot.position.z)
        rospy.loginfo("Target Orientation - (x: %.4f, y: %.4f, z: %.4f, w: %.4f)", Q_obj2robot.orientation.x, Q_obj2robot.orientation.y, Q_obj2robot.orientation.z, Q_obj2robot.orientation.w)
        rospy.sleep(5.0)
        target_pose.position.x = Q_obj2robot.position.x
        target_pose.position.y = Q_obj2robot.position.y
        target_pose.position.z = Q_obj2robot.position.z
        # target_pose.orientation.x = Q_obj2robot.orientation.x
        # target_pose.orientation.y = Q_obj2robot.orientation.y
        # target_pose.orientation.z = Q_obj2robot.orientation.z
        # target_pose.orientation.w = Q_obj2robot.orientation.w

        return target_pose

    def pose_to_matrix(self, pose):
        """
        Convert a geometry_msgs/Pose message into a 4x4 homogeneous transformation matrix.

        Args:
            pose (geometry_msgs.msg.Pose): The input pose containing position (x, y, z)
                                        and orientation (quaternion: x, y, z, w).

        Returns:
            numpy.ndarray: A 4x4 transformation matrix representing the same pose.
        """
        trans = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        T = quaternion_matrix(quat)
        T[0:3, 3] = trans
        return T

    def matrix_to_pose(self, T):
        """
        Convert a 4x4 homogeneous transformation matrix into a geometry_msgs/Pose message.

        Args:
            T (numpy.ndarray): A 4x4 transformation matrix containing position and orientation.

        Returns:
            geometry_msgs.msg.Pose: A Pose message with position (x, y, z) and
                                    orientation (quaternion: x, y, z, w).
        """
        pose = Pose()
        trans = T[0:3, 3]
        quat = quaternion_from_matrix(T)
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def shutdown(self):
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    planner = HandCameraBasedPoseTransformerPlanner()
    try:
        planner.move_action()
    except rospy.ROSInterruptException:
        planner.shutdown()
