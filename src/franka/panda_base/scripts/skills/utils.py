import numpy as np
import json
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

def switch_controllers(start_controllers, stop_controllers, timeout=5.0):
    """
    切换ROS控制器
    """
    rospy.wait_for_service('/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy(
            '/controller_manager/switch_controller', SwitchController)
        req = SwitchControllerRequest()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = 1  # STRICT
        req.start_asap = True
        req.timeout = timeout
        resp = switch_controller(req)
        return resp.ok
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

def perform_insertion():
    success = switch_controllers(
        start_controllers=["cartesian_impedance_example_controller"],
        stop_controllers=["position_joint_trajectory_controller"]
    )
    if not success:
        rospy.logerr("Failed to switch to impedance control!")
        return

def perform_position_trajectory():
    success = switch_controllers(
        start_controllers=["position_joint_trajectory_controller"],
        stop_controllers=["cartesian_impedance_example_controller"]
    )
    if not success:
        rospy.logerr("Failed to switch to position joint trajectory control!")
        return

def normalize_quaternion(q):
    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm
    return q

def send_goal(client, goal):
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    return result

def add_top_poses(known_poses):
    for position in [
        "change_gripper_position", "put_gripper_position",
        "pickup_position_shaft3", "place_position_shaft3",
        "pickup_position_shaft1", "place_position_shaft1",
        "pickup_position_shaft2", "place_position_shaft2",
        "pickup_position_gear3", "place_position_gear3",
        "place_position_gear2", "put_gripper_position",
        "pickup_position_gear1", "place_position_gear1",
    ]:
        known_poses[position + "_top"] = known_poses[position].copy()
        known_poses[position + "_top"].z += 0.12

    for position in ["pickup_position_gear2"]:
        known_poses[position + "_top"] = known_poses[position].copy()
        known_poses[position + "_top"].z += 0.18

    for position in ["change_gripper_position_2"]:
        known_poses[position + "_top"] = known_poses[position].copy()
        known_poses[position + "_top"].z += 0.18

def force_callback(self, msg):
    self.force_get_z = msg.wrench.force.z
    self.force_get_x = msg.wrench.force.x
    self.force_get_y = msg.wrench.force.y
    self.force_z = self.alpha * self.force_get_z + (1 - self.alpha) * self.force_z
    self.force_x = self.alpha * self.force_get_x + (1 - self.alpha) * self.force_x
    self.force_y = self.alpha * self.force_get_y + (1 - self.alpha) * self.force_y

def franka_state_callback(self, msg):
    self.insertion_pose_pre.position.x = msg.O_T_EE[12]
    self.insertion_pose_pre.position.y = msg.O_T_EE[13]
    self.insertion_pose_pre.position.z = msg.O_T_EE[14]
    self.insertion_pose_pre.orientation.x = msg.O_T_EE[0]
    self.insertion_pose_pre.orientation.y = msg.O_T_EE[1]
    self.insertion_pose_pre.orientation.z = msg.O_T_EE[2]
    self.insertion_pose_pre.orientation.w = msg.O_T_EE[3]