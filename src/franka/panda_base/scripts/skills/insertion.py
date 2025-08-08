import rospy
import copy
from math import sin, pi
from geometry_msgs.msg import PoseStamped

from skills.utils import normalize_quaternion, perform_insertion, perform_position_trajectory
from skills.motion import move_down
from skills.lissajous import lissajous

def insertion(move_group, pub, initial_pose, amplitude_multi, deepth, 
              A_x, A_y, freq_x, freq_y, phase, force_x, force_y, force_z):
    """
    Perform an insertion skill with sinusoidal motion in x/y and downward movement in z.
    Args:
        move_group: MoveIt! commander interface for controlling the robot.
        pub: ROS publisher for PoseStamped messages.
        initial_pose: The starting pose for insertion.
        amplitude_multi: Amplitude multiplier for sinusoidal motion.
        deepth: Depth to insert (not used directly here).
        A_x, A_y: Amplitudes for x and y sinusoidal motion.
        freq_x, freq_y: Frequencies for x and y sinusoidal motion.
        phase: Phase offset for x sinusoidal motion.
        force_x, force_y, force_z: Forces (not used directly here).
    """
    wpose = move_group.get_current_pose().pose
    initial_pose = copy.deepcopy(initial_pose)
    
    rospy.loginfo("Start Inserting!")
    perform_insertion()

    rate = rospy.Rate(10)
    t = 0.0

    current_pose = move_group.get_current_pose().pose
    target_z = wpose.position.z - 0.008  # Target z position for insertion

    # Move down along z with superimposed sinusoidal motion in x and y
    while (not rospy.is_shutdown()) and (current_pose.position.z > target_z):
        current_pose = move_group.get_current_pose().pose
        rospy.loginfo("Still need to go down along z: %s m", current_pose.position.z - target_z)

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"

        # 使用李萨如曲线 skill 生成 x/y
        msg.pose.position.x = lissajous(initial_pose.position.x, amplitude_multi, A_x, freq_x, t, phase)
        msg.pose.position.y = lissajous(initial_pose.position.y, amplitude_multi, A_y, freq_y, t, 0)
        msg.pose.position.z = initial_pose.position.z - 0.0085

        # Maintain initial orientation
        msg.pose.orientation.x = initial_pose.orientation.x
        msg.pose.orientation.y = initial_pose.orientation.y
        msg.pose.orientation.z = initial_pose.orientation.z
        msg.pose.orientation.w = initial_pose.orientation.w
        msg.pose.orientation = normalize_quaternion(msg.pose.orientation)

        pub.publish(msg)
        t += 1.0 / 10.0
        rate.sleep()

    perform_position_trajectory()
    rospy.sleep(2)

    # Update pose to match final insertion position
    update_pose = move_group.get_current_pose().pose
    update_pose.position.x = wpose.position.x
    update_pose.position.y = wpose.position.y
    update_pose.position.z = current_pose.position.z
    update_pose.orientation.x = wpose.orientation.x
    update_pose.orientation.y = wpose.orientation.y
    update_pose.orientation.z = wpose.orientation.z
    update_pose.orientation.w = wpose.orientation.w

    (plan, _) = move_group.compute_cartesian_path([update_pose], 0.01)
    move_group.execute(plan, wait=True)
    
    move_group.move_down(deepth)