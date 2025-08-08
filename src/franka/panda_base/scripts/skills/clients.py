import rospy
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction
from franka_gripper.msg import GraspAction, MoveAction

def start_clients(self):
    actions = {
        '/position_joint_trajectory_controller/follow_joint_trajectory': FollowJointTrajectoryAction,
        '/franka_gripper/grasp': GraspAction,
        '/franka_gripper/move': MoveAction
    }

    client_names = ["joint_trajectory", "grasp", "move_gripper"]
    self.clients = {}

    for i, action in enumerate(actions):
        action_client = SimpleActionClient(action, actions[action])
        rospy.loginfo("Waiting for '" + action + "' action to come up")
        action_client.wait_for_server()
        self.clients[client_names[i]] = action_client