#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
import roslaunch
import os, sys
import rospkg  

rospack = rospkg.RosPack()
PANDA_PKG_PATH = rospack.get_path('panda_base')
FRANKA_LAUNCH = os.path.join(PANDA_PKG_PATH, 'launch', 'panda_with_impedance_controller.launch')
UR5E_PKG_PATH = rospack.get_path('ur5e_control')
UR5E_LAUNCH = os.path.join(UR5E_PKG_PATH, 'launch', 'ur5e_control.launch')

# Add scripts directory to sys.path
sys.path.insert(0, os.path.join(rospack.get_path('panda_base'), 'scripts'))
sys.path.insert(0, os.path.join(rospack.get_path('ur5e_control'), 'scripts'))

try:
    from panda_control import Franka_control
except Exception as e:
    rospy.logerr(f"Import Franka_control failed: {e}.")
    Franka_control = None

try:
    from ur5e_control import UR5eController
except Exception as e:
    rospy.logerr(f"Import UR5eController failed: {e}.")
    UR5eController = None

class RobotInterface:
    """
    Unified robot interface for Franka and UR5e.
    """
    def __init__(self, robot_type, **kwargs):
        if not rospy.core.is_initialized():
            rospy.init_node('robot_interface_node', anonymous=True)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        if robot_type.lower() == 'franka':
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, [FRANKA_LAUNCH])
        elif robot_type.lower() == 'ur5e':
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, [UR5E_LAUNCH])
        else:
            raise ValueError(f"Unsupported robot_type: {robot_type}")
        rospy.loginfo("[Interface] Launching controller nodes...")
        self.launch.start()
        if robot_type.lower() == 'ur5e':
            rospy.loginfo("[Interface] Waiting 15 seconds for ROS nodes to fully start...")
            rospy.sleep(15)
        else:
            rospy.sleep(2)

        self.robot_type = robot_type.lower()
        if self.robot_type == 'franka':
            if Franka_control is None:
                raise ImportError("Franka_control not found")
            rospy.loginfo("[Interface] Initializing Franka controller...")
            self.robot = Franka_control(**kwargs)
        elif self.robot_type == 'ur5e':
            if UR5eController is None:
                raise ImportError("UR5eController not found. Check package and path.")
            rospy.loginfo("[Interface] Initializing UR5e controller...")
            self.robot = UR5eController(**kwargs)
        else:
            raise ValueError(f"Unsupported robot_type: {robot_type}")

        planlist_path = os.path.join(os.path.dirname(__file__), 'planlist.json')
        try:
            with open(planlist_path, 'r', encoding='utf-8') as f:
                self.planlist = json.load(f)
            rospy.loginfo(f"[Interface] Loaded planlist.json, tasks: {list(self.planlist.keys())}")
        except Exception as e:
            rospy.logerr(f"[Interface] Failed to load planlist.json: {e}")
            self.planlist = {}

    def _map_planlist_step_to_skills(self, step):
        """
        Map a planlist step to robot skills.
        """
        skills = []
        if len(step) == 1:
            skills.append("go_to")
        elif len(step) == 2:
            skills.append("go_to")
            action_map = {
                "screw": "screw",
                "rotation": "rotate_angle",
                "gear_engagement_1": ["insertion", "gear_engagement", "open_gripper"],
                "gear_engagement_2": ["gear_engagement", "release_gripper"],
                "grasp": "grasp",
                "grasp_without_gripper": "grasp_without_gripper",
                "open_gripper": "open_gripper",
                "change_gripper": "change_gripper",
                "insertion_1": ["insertion", "release_gripper"],
                "insertion_2": ["insertion", "screw", "open_gripper"],
                "insertion_3": ["insertion", "release_gripper"],
                "move_down": "move_down",
                "homing_joint_position_1": "homing_joint_position",
                "homing_joint_position_2": "homing_joint_position",
                "release_gripper": "release_gripper"
            }
            gripper_action = step[1]
            mapped = action_map.get(gripper_action)
            if mapped:
                if isinstance(mapped, list):
                    skills.extend(mapped)
                else:
                    skills.append(mapped)
            else:
                skills.append(gripper_action)
        return skills

    def execute(self, action, *args, **kwargs):
        """
        Call the underlying controller's method.
        """
        rospy.loginfo(f"[Interface] Executing action: {action} on {self.robot_type}, args: {args} {kwargs}")
        if not hasattr(self.robot, action):
            raise AttributeError(f"Robot does not support action: {action}")
        method = getattr(self.robot, action)
        result = method(*args, **kwargs)
        if result is False:
            rospy.logwarn(f"Action {action} did not complete successfully")
            raise RuntimeError(f"Action {action} execution failed")
        rospy.loginfo(f"Action {action} completed successfully")
        return result

    def can_execute_task(self, task_name):
        """
        Check if the robot supports all required skills for the task.
        """
        if task_name not in self.planlist:
            rospy.logwarn(f"[Interface] Task {task_name} not found in planlist")
            return False
        kb_path = os.path.join(os.path.dirname(__file__), 'global_knowledge_base.json')
        try:
            with open(kb_path, 'r', encoding='utf-8') as f:
                kb = json.load(f)
        except Exception as e:
            rospy.logerr(f"[Interface] Failed to load global_knowledge_base.json: {e}")
            return False
        supported = set(kb.get(self.robot_type, {}).get("skills", []))
        for step in self.planlist[task_name]:
            for skill in self._map_planlist_step_to_skills(step):
                if skill not in supported:
                    rospy.logwarn(f"[Interface] Robot '{self.robot_type}' does not support skill: {skill}")
                    return False
        return True

    def execute_plan(self, task_name):
        """
        Call the robot's plan_and_execute method.
        """
        rospy.loginfo(f"[Interface] Calling task:{task_name}")
        if not hasattr(self.robot, "plan_and_execute"):
            raise AttributeError("Robot does not support plan_and_execute method")
        self.robot.plan_and_execute(plan_name=task_name)
        rospy.loginfo(f"[Interface] Task {task_name} completed")

    def shutdown(self):
        rospy.loginfo("[Interface] Shutting down controller nodes...")
        self.launch.shutdown()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python interface.py <robot_type> <action|execute_plan> [task_name or action args]")
        sys.exit(1)

    robot_type = sys.argv[1]
    action     = sys.argv[2]
    raw_args   = sys.argv[3:]

    # Parse positional and keyword arguments
    pos, kw = [], {}
    i = 0
    while i < len(raw_args):
        if raw_args[i].startswith('--'):
            k = raw_args[i][2:]
            v = raw_args[i+1]
            try:    v = int(v)
            except: 
                try: v = float(v)
                except: pass
            kw[k] = v
            i += 2
        else:
            v = raw_args[i]
            try:    v = int(v)
            except:
                try: v = float(v)
                except: pass
            pos.append(v)
            i += 1

    iface = RobotInterface(robot_type)
    try:
        if action == 'execute_plan' and pos:
            # Check knowledge base before execution
            if iface.can_execute_task(pos[0]):
                rospy.loginfo(f"{robot_type} supports all required skills for task {pos[0]}")
                iface.execute_plan(pos[0])
            else:
                rospy.logerr(f"Robot {robot_type} does not support all required skills for task {pos[0]}")
        else:
            iface.execute(action, *pos, **kw)
    except Exception as e:
        rospy.logerr(f"Execution failed: {e}")
    finally:
        iface.shutdown()
