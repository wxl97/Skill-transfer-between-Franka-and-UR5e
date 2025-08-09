#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os

# rrt_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '/../'))

# Add RRT path to sys.path
# if rrt_path not in sys.path:
#     sys.path.insert(0, rrt_path)

import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from RRT.rrt_with_sobol_sampler import RRTSobol
from geometry_msgs.msg import Pose
from moveit_commander import RobotCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest


class RRTPlanner:
    def __init__(
        self,
        joint_names=None,
        controller_action='/position_joint_trajectory_controller/follow_joint_trajectory',
        rand_area=None,
        robot_radius=0.05,
        goal_sample_rate=20,
        path_resolution=0.05,
        client=None,
        move_group=None
    ):
        """
        Initialize the trajectory executor
        :param joint_names: List of joint names (default: Panda 7 DOF)
        :param controller_action: Controller action name
        :param rand_area: Sampling space
        :param robot_radius: Robot radius
        :param goal_sample_rate: Goal sampling rate
        :param path_resolution: Path resolution
        :param client: Optional, external actionlib client
        """
        if joint_names is None:
            joint_names = [
                'panda_joint1', 'panda_joint2', 'panda_joint3',
                'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
            ]
        if rand_area is None:
            rand_area = [-3.14, 3.14]

        self.move_group = move_group
        self.joint_names = joint_names
        self.controller_action = controller_action
        self.rand_area = rand_area
        self.robot_radius = robot_radius
        self.goal_sample_rate = goal_sample_rate
        self.path_resolution = path_resolution

        if client is not None:
            self.client = client
            self.external_client = True
        else:
            self.client = actionlib.SimpleActionClient(
                controller_action, FollowJointTrajectoryAction)
            self.external_client = False

    def wait_for_server(self):
        if not self.external_client:
            rospy.loginfo("Waiting for controller server...")
            self.client.wait_for_server()
            rospy.loginfo("Controller server connected.")

    @staticmethod
    def create_trajectory(points, joint_names, time_step=1.0):
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        for i, point in enumerate(points):
            traj_point = JointTrajectoryPoint()
            traj_point.positions = point
            traj_point.time_from_start = rospy.Duration.from_sec(i * time_step)
            trajectory.points.append(traj_point)
        return trajectory

    def send_trajectory(self, trajectory):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration.from_sec(2.0)
        rospy.loginfo("Sending trajectory to controller...")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo(f"Trajectory execution completed, result: {result}")

    def ik_solver(self, pose):
        rospy.wait_for_service('/compute_ik')
        try:
            ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
            ik_request = GetPositionIKRequest()
            ik_request.ik_request.group_name = group_name
            ik_request.ik_request.pose_stamped.header.frame_id = frame_id
            ik_request.ik_request.pose_stamped.pose = pose
            ik_request.ik_request.timeout = rospy.Duration(timeout)

            robot = RobotCommander()
            ik_request.ik_request.robot_state.joint_state = robot.get_current_state().joint_state

            ik_response = ik_service(ik_request)

            if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                joint_positions = ik_response.solution.joint_state.position[:7]  # Only take the first 7 joints
                rospy.loginfo("Computed joint positions: %s", joint_positions)
                return joint_positions
            else:
                rospy.logerr("IK failed with error code: %d" % ik_response.error_code.val)
                return None

        except rospy.ServiceException as e:
            rospy.logerr("IK service call failed: %s" % e)
            return None
    
    def plan_and_execute(
        self,
        start_pose,
        target_pose,
        obstacle_list=None,
        time_step=1.0,
    ):
        """
        Plan and execute a joint-space trajectory
        :param start_pose: Start pose (joint angles or Cartesian pose)
        :param target_pose: Target pose (joint angles or Cartesian pose)
        :param obstacle_list: List of obstacles
        :param time_step: Time interval between trajectory points
        :return: True/False
        """
        if obstacle_list is None:
            obstacle_list = []

        start_joint = self.ik_solver(start_pose)
        if start_joint is None:
            rospy.logerr("Failed to compute IK for start pose!")
            return False
        target_joint = self.ik_solver(target_pose)
        if target_joint is None:
            rospy.logerr("Failed to compute IK for target pose!")
            return False

        rrt = RRTSobol(
            start=start_joint,
            goal=target_joint,
            rand_area=self.rand_area,
            obstacle_list=obstacle_list,
            robot_radius=self.robot_radius,
            goal_sample_rate=self.goal_sample_rate,
            path_resolution=self.path_resolution
        )
        path = rrt.planning()
        if not path:
            rospy.logerr("Path planning failed!")
            return False

        trajectory = self.create_trajectory(path, self.joint_names, time_step)
        self.send_trajectory(trajectory)
        return True

# Support direct execution for testing
if __name__ == "__main__":
    rospy.init_node("track_rrt_trajectory_class", anonymous=True)
    start_pose = [-0.594591, 0.430150, 0.136105, -1.626641, -0.012580, 2.060708, 0.338735]
    target_pose = [-0.535040, -0.676063, 0.394390, -2.398527, 0.256021, 1.779811, 0.522896]
    obstacle_list = [
        [0.45, -0.15, 0.45, -0.92, 0.39, -0.01, 0.01, 0.02],
        [0.55, -0.25, 0.40, -0.92, 0.39, -0.01, 0.02, 0.02],
    ]
    executor = RRTPlanner()
    executor.wait_for_server()
    result = executor.plan_and_execute(start_pose, target_pose, obstacle_list=obstacle_list)
    print("Trajectory execution result:", result)
