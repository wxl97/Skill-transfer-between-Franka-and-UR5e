import rospy
from math import radians
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from skills.utils import send_goal

def rotate_angle(move_group, client, angle, duration=5.0, acc=0.0):
    """
    Rotates the 7th joint of the robot arm by a specified angle.

    Args:
        move_group: The MoveGroupCommander object for the robot.
        client: The action client to send the trajectory goal.
        angle: The angle (in degrees) to rotate the 7th joint.
        duration: Duration (in seconds) for the rotation motion.
        acc: Acceleration for the 7th joint.
    """
    current_joint_positions = move_group.get_current_joint_values()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        'panda_joint1', 'panda_joint2', 'panda_joint3',
        'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
    ]
    point = JointTrajectoryPoint()
    point.positions = current_joint_positions
    point.positions[6] += radians(angle)
    point.time_from_start = rospy.Duration(duration)
    point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, acc]
    point.velocities = [0.0] * 7
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
    send_goal(client, goal)
    rospy.sleep(1.0)


def screw(move_group, client, open_gripper, grasp):
    """
    Executes a screw-like operation by alternating gripper open/close and rotating the 7th joint.

    Args:
        open_gripper: Function to open the gripper.
        grasp: Function to close the gripper (grasp).
    """
    ''' Execute the place with rotation operation 6 times. '''
    for i in range(6): 
        open_gripper()
        rotate_angle(move_group, client, -90, duration=8.0)
        grasp()
        rotate_angle(move_group, client, 90, duration=8.0)
    open_gripper()