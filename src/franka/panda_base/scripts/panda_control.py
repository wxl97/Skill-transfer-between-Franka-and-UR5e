#!/usr/bin/env python
from __future__ import print_function
import os
import sys
sys.path.insert(0, "/home/blackbird/xinlong/transfer_ws/src/franka/panda_base/scripts")

import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import subprocess
import json
import numpy as np

# from skills.RRTPlanner import RRTPlanner
from skills.robot_state import get_current_joint_positions, read_current_pose
from skills.utils import force_callback, franka_state_callback, add_top_poses, perform_insertion, perform_position_trajectory
from skills.controller_utils import impedance_control_mode, position_control_mode
from skills.gripper_action import (
    grasp as skill_grasp,
    grasp_without_gripper as skill_grasp_without_gripper,
    open_gripper as skill_open_gripper,
    change_gripper as skill_change_gripper,
    release_gripper as skill_release_gripper
)
from skills.insertion import insertion
from skills.motion import go_to, move_on_direction, move_down, homing_joint_position, move_to_safe_pose
from skills.rotation import rotate_angle, screw
from skills.gear_engagement import gear_engagement
from skills.goal import Goal
from skills.plan import Plan, PlanEl
from skills.plan_execute import plan_and_execute, load_plan
from skills.clients import start_clients

from six.moves import input
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal, GraspEpsilon
from actionlib import SimpleActionClient
from math import radians, pi, sin
from geometry_msgs.msg import PoseStamped,Pose
from tf.transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

        
class Franka_control(object):

    def __init__(self):
        super(Franka_control, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        

        # Provides information such as the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # This provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # This object is an interface to a planning group (group of joints). This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Publisher used to display trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = move_group.get_planning_frame()
        self.eef_link = move_group.get_end_effector_link()
        self.group_names = robot.get_group_names()
        self.n_executions = 0

        self.start_clients()
        
        # Insertion pre-pose
        self.insertion_pose_pre = Pose()


        rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.force_callback, tcp_nodelay=True)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.franka_state_callback, tcp_nodelay=True)
        self.pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
        
        # Read JSON database
        config_path = os.path.join(os.path.dirname(__file__), "panda_config.json")
        with open(config_path, "r") as f:
            config = json.load(f)
        self.home_joint_positions = config["home_joint_positions"]
        self.known_poses_or = {}
        for name, values in config["known_poses"].items():
            self.known_poses_or[name] = Goal(*values)

        # Read filter parameters
        filter_params = config.get("filter_params", {})
        self.alpha = filter_params.get("alpha", 0.05)
        self.force_z_filtered = 0.0
        self.force_x_filtered = 0.0
        self.force_y_filtered = 0.0
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0

        # Read insertion parameters
        insertion_params = config.get("insertion_params", {})
        self.A_x = insertion_params.get("A_x", 0.02)
        self.A_y = insertion_params.get("A_y", 0.03)
        self.freq_x = insertion_params.get("freq_x", 0.8)
        self.freq_y = insertion_params.get("freq_y", 1.0)
        self.phase = insertion_params.get("phase", pi / 2.0)
        self.k_z = insertion_params.get("k_z", 0.0001)
        self.D_x = insertion_params.get("D_x", 0.1)
        self.D_y = insertion_params.get("D_y", 0.1)

        # Read gripper parameters
        gripper_params = config.get("gripper_params", {})
        self.grasp_width_without_gripper = gripper_params.get("GRASP_WIDTH_WITHOUT_GRIPPER", 0.024)
        self.grasp_width_gripper = gripper_params.get("GRASP_WIDTH_GRIPPER", 0.028)
        self.grasp_speed = gripper_params.get("GRASP_SPEED", 0.05)
        self.grasp_with_gripper_force = gripper_params.get("GRASP_WITH_GRIPPER_FORCE", 60.0)
        self.grasp_object_force = gripper_params.get("GRASP_OBJECT_FORCE", 40.0)
        self.gripper_width_gripper = gripper_params.get("GRIPPER_WIDTH_GRIPPER", 0.08)
        self.gripper_speed = gripper_params.get("GRIPPER_SPEED", 0.05)
        self.change_gripper_width_gripper = gripper_params.get("CHANGE_GRIPPER_WIDTH_GRIPPER", 0.043)
        self.release_gripper_width_gripper = gripper_params.get("RELEASE_GRIPPER_WIDTH_GRIPPER", 0.08)

        self.Fz_threshold = 1.0  # If needed, can also be configured in config

        self.known_poses = copy.deepcopy(self.known_poses_or)

    def start_clients(self):
        start_clients(self)

    def franka_state_callback(self, msg):
        franka_state_callback(self, msg)

    def impedance_control_mode(self):
        impedance_control_mode()

    def position_control_mode(self):
        position_control_mode()

    def add_top_poses(self):
        add_top_poses(self.known_poses)

    def homing_joint_position(self, pose):
        homing_joint_position(self.move_group, self.home_joint_positions, pose)

    def force_callback(self, msg):
        force_callback(self, msg)

    def homing(self):
        self.go_to("start")

    def go_to(self, pose_name):
        go_to(self.move_group, self.known_poses, pose_name, self.clients["joint_trajectory"])

    def go_to_with_RRT(self, start_pose, target_pose, obstacle_list=None, time_step=1.0):
        """
        Use RRT for joint space trajectory planning and execution
        :param start_pose: Starting joint angles (list of length 7)
        :param target_pose: Target joint angles (list of length 7)
        :param obstacle_list: List of obstacles (optional)
        :param time_step: Time interval for interpolation between points
        """
        # If a client already exists, it can be passed directly; otherwise, it is created by RRTPlanner by default
        planner = RRTPlanner(client=self.clients.get("joint_trajectory", None))
        result = planner.plan_and_execute(
            start_pose=start_pose,
            target_pose=target_pose,
            obstacle_list=obstacle_list,
            time_step=time_step,
            move_group=self.move_group
        )
        if not result:
            rospy.logerr("RRT trajectory planning and execution failed!")
        else:
            rospy.loginfo("RRT trajectory successfully executed.")


    def plan_and_execute(self, plan_name="gear_assembling"):
        plan_and_execute(self, plan_name)
        
    def move_on_direction(self, direction, length):
        move_on_direction(self.move_group, direction, length)

    def move_down(self, length):
        move_down(self.move_group, length)

    def insertion(self, amplitude_multi, deepth):
        insertion(
            self.move_group,
            self.pub,
            self.insertion_pose_pre,
            amplitude_multi,
            deepth,
            self.A_x,
            self.A_y,
            self.freq_x,
            self.freq_y,
            self.phase,
            self.force_x,
            self.force_y,
            self.force_z
        )

    def grasp(self, width=None, speed=None, force=None):
        '''Grasp an object.'''
        if width is None:
            width = self.grasp_width_gripper
        if speed is None:
            speed = self.grasp_speed
        if force is None:
            force = self.grasp_with_gripper_force
        skill_grasp(width, speed, force, self.clients["grasp"])

    def grasp_without_gripper(self, width=None, speed=None, force=None):
        '''Grasp an object without gripper.'''
        if width is None:
            width = self.grasp_width_without_gripper
        if speed is None:
            speed = self.grasp_speed
        if force is None:
            force = self.grasp_object_force
        skill_grasp_without_gripper(width, speed, force, self.clients["grasp"])

    def open_gripper(self, width=None, speed=None):
        '''Release the object.'''
        if width is None:
            width = self.change_gripper_width_gripper
        if speed is None:
            speed = self.gripper_speed
        skill_open_gripper(width, speed, self.clients["move_gripper"])

    def change_gripper(self, width=None, speed=None):
        '''Move to the special position to grab the gripper.'''
        if width is None:
            width = self.change_gripper_width_gripper
        if speed is None:
            speed = self.grasp_speed
        skill_change_gripper(width, speed, self.clients["move_gripper"])

    def release_gripper(self, width=None, speed=None):
        '''Move to the special position to release the gripper.'''
        if width is None:
            width = self.release_gripper_width_gripper
        if speed is None:
            speed = self.gripper_speed
        skill_release_gripper(width, speed, self.clients["move_gripper"])

    def rotate_angle(self, angle, duration=5.0, acc=0.0):
        rotate_angle(self.move_group, self.clients["joint_trajectory"], angle, duration, acc)

    def gear_engagement(self):
        gear_engagement(self.move_group, self.force_get_z, self.force_z, self.move_down, self.rotate_angle)

    def screw(self):
        screw(self.move_group, self.clients["joint_trajectory"] ,self.open_gripper, self.grasp)

    def get_current_joint_positions(self):
        return get_current_joint_positions(self.move_group)

    def read_current_pose(self):
        return read_current_pose(self.move_group)

    def load_plan(self, plan_name="all"):
        return load_plan(self, plan_name)
    
    def move_to_safe_pose(self, height=0.12):
        move_to_safe_pose(self.move_group, height)


def main():
    try:
        interface = Franka_control()
        mode = rospy.get_param("~mode", "all")
        # support shaft1/shaft2/shaft3/gear1/gear2/gear3/gear_assembling
        if mode in ["homing", "shaft1", "shaft2", "shaft3", "gear1", "gear2", "gear3", "gear_assembling"]:
            rospy.loginfo(f"Executing plan: {mode}")
            interface.plan_and_execute(plan_name=mode)
        elif mode == "demo-2":
            rospy.loginfo("Reading current pose and gripper state...")
            interface.read_current_pose()
        elif mode == "demo-3":
            rospy.loginfo("Reading current joint positions...")
            interface.get_current_joint_positions()
        elif mode == "move_up":
            interface.move_down(-0.05)
        elif mode == "init_pose":
            interface.homing_joint_position("1")
        elif mode == "rotate_joint":
            interface.screw()
        elif mode == "gear_engagement":
            interface.gear_engagement() 
        else:
            rospy.logwarn("Unknown mode specified, no action taken.")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    


if __name__ == "__main__":
    rospy.init_node("franka_controller", anonymous=True)
    main()
