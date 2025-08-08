from franka_gripper.msg import MoveGoal, GraspGoal, GraspEpsilon

def grasp(grasp_width, grasp_speed, grasp_force, client):
    """
    Close the gripper to grasp an object with specified width, speed, and force.
    """
    epsilon = GraspEpsilon(inner=0.005, outer=0.005)
    goal = GraspGoal(width=grasp_width, epsilon=epsilon, speed=grasp_speed, force=grasp_force)
    client.send_goal(goal)
    client.wait_for_result()

def grasp_without_gripper(grasp_width, grasp_speed, grasp_force, client):
    """
    Simulate a grasp action without using the physical gripper, with specified width, speed, and force.
    """
    epsilon = GraspEpsilon(inner=0.01, outer=0.01)
    goal = GraspGoal(width=grasp_width, epsilon=epsilon, speed=grasp_speed, force=grasp_force)
    client.send_goal(goal)
    client.wait_for_result()

def open_gripper(change_gripper_width, gripper_speed, client):
    """
    Open the gripper to a specified width at a given speed.
    """
    epsilon = GraspEpsilon(inner=0.01, outer=0.01)
    goal = GraspGoal(width=change_gripper_width, epsilon=epsilon, speed=gripper_speed)
    client.send_goal(goal)
    client.wait_for_result()

def change_gripper(change_gripper_width, grasp_speed, client):
    """
    Change the gripper width to a specified value at a given speed.
    """
    epsilon = GraspEpsilon(inner=0.01, outer=0.01)
    goal = GraspGoal(width=change_gripper_width, epsilon=epsilon, speed=grasp_speed)
    client.send_goal(goal)
    client.wait_for_result()

def release_gripper(release_gripper_width, gripper_speed, client):
    """
    Release the gripper to a specified width at a given speed using MoveGoal.
    """
    goal = MoveGoal(width=release_gripper_width, speed=gripper_speed)
    client.send_goal(goal)
    client.wait_for_result()