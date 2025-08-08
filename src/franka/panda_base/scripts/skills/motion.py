import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from skills.utils import send_goal

def go_to(move_group, known_poses, pose_name, client):
    """
    Move the robot to a known pose.
    This function plans a Cartesian path to a target pose specified by pose_name in known_poses,
    then sends the trajectory to the controller for execution.
    :param move_group: MoveIt! move_group object
    :param known_poses: Dictionary of known poses
    :param pose_name: Name of the target pose (string)
    :param client: Action client for trajectory execution
    """

    
    # move_on_direction(move_group, "z", 0.12)  # Move up before going to the pose
    
    pose = known_poses[pose_name]
    wpose = move_group.get_current_pose().pose
    wpose.position.x, wpose.position.y, wpose.position.z = pose.x, pose.y, pose.z
    wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w = pose.dx, pose.dy, pose.dz, pose.dw
    
    (plan, _) = move_group.compute_cartesian_path([wpose], 0.01)
    goal = FollowJointTrajectoryGoal(trajectory=plan.joint_trajectory, goal_time_tolerance=rospy.Duration.from_sec(2))
    send_goal(client, goal)




def move_on_direction(move_group, direction, length):
    """
    Move the robot end-effector along a specified direction (x, y, or z) by a given length.
    The function computes a new pose offset in the given direction and executes the Cartesian path.
    :param move_group: MoveIt! move_group object
    :param direction: Direction to move ('x', 'y', or 'z')
    :param length: Distance to move along the specified direction
    """
    wpose = move_group.get_current_pose().pose
    if direction == "x":
        wpose.position.x += length
    elif direction == "y":
        wpose.position.y += length
    elif direction == "z":
        wpose.position.z += length
    else:
        rospy.logerr("Wrong direction!")
    (plan, _) = move_group.compute_cartesian_path([wpose], 0.01)
    move_group.execute(plan, wait=True)

def move_down(move_group, length):
    """
    Move the robot end-effector downwards along the z-axis by a specified length.
    This is a specialized version of move_on_direction for the negative z direction.
    :param move_group: MoveIt! move_group object
    :param length: Distance to move down along the z-axis
    """
    move_group.set_max_velocity_scaling_factor(0.2)
    move_group.set_max_acceleration_scaling_factor(0.2)
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= length
    (plan, _) = move_group.compute_cartesian_path([wpose], 0.01)
    move_group.execute(plan, wait=True)

def homing_joint_position(move_group, home_joint_positions, pose):
    """
    Move the robot to a predefined home joint position.
    The function sets the joint values to the specified home pose and executes the motion.
    :param move_group: MoveIt! move_group object
    :param home_joint_positions: Dictionary of home joint positions
    :param pose: Name of the home pose (string)
    """
    positions = home_joint_positions.get(str(pose))
    if positions is None:
        import rospy
        rospy.logerr("Unknown home pose: %s", pose)
        return
    move_group.set_joint_value_target(positions)
    move_group.go(wait=True)
    move_group.stop()

def move_to_safe_pose(move_group, height = 0.12):
    """
    Move the robot to a safe pose at a specified height above the current position.
    """
    move_on_direction(move_group, "z", height)
