#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import numpy as np
import tf.transformations as tf_trans
import copy
from geometry_msgs.msg import Pose
from math import radians
from geometry_msgs.msg import WrenchStamped
import socket
import time 
from scipy.signal import butter, lfilter
from control_mode_switcher import switch_to_local_control, switch_to_remote_control
from ur5e_impedance_controller import impedance_control

from skills.goal import Goal
from skills.plan import Plan, PlanEl
from skills.plan_execute import plan_and_execute, load_plan

class UR5eController:
    def __init__(self):
        super(UR5eController, self).__init__()
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("ur5e_controller", anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize MoveGroup
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")

        # Force threshold for insertion (Z-axis)
        self.Fz_threshold = 0.6

        # Compliance gain along Z during insertion
        self.k_z = 0.005

        # Damping parameters
        self.D_x = 0.1  # X-axis damping coefficient
        self.D_y = 0.1  # Y-axis damping coefficient
        
        # Pushing gain
        self.k = 0.02

        # Force sensor values
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        rospy.Subscriber("/wrench", WrenchStamped, self.force_callback, tcp_nodelay=True)

        # Robotiq gripper TCP connection
        HOST = "172.16.15.10"  # Replace with UR robot IP
        PORT = 63352           # Robotiq TCP port
        self.robotiq = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.robotiq.connect((HOST,PORT))

        # Activate gripper (optional)
        # self.robotiq.sendall(b'SET ACT 1\n') 

    def gripperMove(self, position = 0, speed = 50, force = 50):
        """
        Activate and control the Robotiq gripper.
        Args:
            position (int): Opening position [0-255], default 0
            speed (int): Speed [0-255], default 50
            force (int): Force [0-255], default 50
        """
        # self.robotiq.sendall(b'SET ACT 1\n')  # activation if needed
        self.robotiq.sendall(b'SET GTO 0\n')    # reset motion
        self.robotiq.sendall(b'SET MOD 1\n')    # set priority mode
        self.robotiq.sendall(f'SET POS {position}\n'.encode())  # set position
        self.robotiq.sendall(f'SET SPE {speed}\n'.encode())     # set speed
        self.robotiq.sendall(f'SET FOR {force}\n'.encode())     # set force
        self.robotiq.sendall(b'SET GTO 1\n')    # execute motion
        print(f"Pos={position}, Speed={speed}, Force={force}")

        time.sleep(2) 

    def getPosition(self):
        """Query current gripper position (raw)."""
        self.robotiq.sendall(b'GET POS\n')
        self.data = self.robotiq.recv(2**10)
        print("Current gripper position (raw socket object):", self.robotiq)

    def move_to_target(self, target_position):
        """
        Plan and execute a motion with MoveIt to a target position (keep default orientation).
        Args:
            target_position (tuple): (x, y, z) in base_link frame
        """
        target_pose = Pose()
        target_pose.position.x = target_position[0]
        target_pose.position.y = target_position[1]
        target_pose.position.z = target_position[2]
        target_pose.orientation.w = 1.0  # keep default orientation

        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()

        if plan[0]:
            self.move_group.execute(plan[1], wait=True)
            rospy.loginfo("Motion to target executed successfully")
        else:
            rospy.logwarn("Motion planning failed")

        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def cartesian_move(self, pose):
        """
        Execute a Cartesian path to the target pose.
        Args:
            pose (tuple): (x, y, z, qx, qy, qz, qw) in base_link frame
        """
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        
        # Use current pose as a base and set target values
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x, wpose.position.y, wpose.position.z = pose[0], pose[1], pose[2]
        wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w = pose[3], pose[4], pose[5], pose[6]

        (plan, fraction) = self.move_group.compute_cartesian_path([wpose], 0.01)

        if fraction > 0.1:
            self.move_group.execute(plan, wait=True)
            rospy.loginfo("Cartesian path executed successfully")
        else:
            rospy.logwarn("Cartesian path execution failed: only {:.2f}% completed".format(fraction * 100))

    def cartesian_relative_move(self, ch, dis):
        """Relative Cartesian move along x/y/z by distance `dis`."""
        wpose = self.move_group.get_current_pose().pose

        if ch == 'x':
            pose = (wpose.position.x + dis, wpose.position.y, wpose.position.z,
                    wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w)
        elif ch == 'y':
            pose = (wpose.position.x, wpose.position.y + dis, wpose.position.z,
                    wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w)
        elif ch == 'z':
            pose = (wpose.position.x, wpose.position.y, wpose.position.z + dis,
                    wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w)
        else:
            rospy.logwarn("Invalid direction!")
            return

        self.cartesian_move(pose)

    def add_safe_pose(self, pose):
        """
        Move to a safe pose above the given pose (add +0.15m on Z).
        Args:
            pose (tuple): (x, y, z, qx, qy, qz, qw)
        """
        safe_pose = (pose[0], pose[1], pose[2] + 0.15, pose[3], pose[4], pose[5], pose[6])
        self.cartesian_move(safe_pose)

    def safe_move(self):
        """Lift current end-effector pose by +0.1m on Z for safety."""
        wpose = self.move_group.get_current_pose().pose
        safe_pose = (wpose.position.x, wpose.position.y, wpose.position.z + 0.1,
                     wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w)
        self.cartesian_move(safe_pose)

    def move_to_target_pose(self, target_position):
        """Go via a safe pose, then to the target pose."""
        self.add_safe_pose(target_position)
        self.cartesian_move(target_position)

    def homing(self):
        """
        Return UR5e to a predefined home joint configuration.
        """
        home_joint_values = [radians(90), radians(-90), radians(90), radians(-90), radians(-90), 0]
        self.move_group.set_joint_value_target(home_joint_values)

        plan = self.move_group.plan()
        if plan and plan[0]:
            rospy.loginfo("Moving to home position...")
            self.move_group.execute(plan[1], wait=True)
        else:
            rospy.logwarn("Failed to plan home position!")

        self.move_group.stop()

    def shutdown(self):
        """Shutdown MoveIt cleanly."""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("MoveIt shutdown complete")

    def get_current_pose(self):
        """Print current end-effector pose."""
        wpose = self.move_group.get_current_pose().pose
        print(wpose)

    def rotate_joint(self, angle = 90, joint_index = 6):
        """Rotate a given joint by `angle` degrees (1-based index)."""
        self.move_group.set_max_velocity_scaling_factor(0.5) 
        self.move_group.set_max_acceleration_scaling_factor(0.5) 

        print("Start rotating joint ", joint_index, " by ", angle, " degrees")
        current_joint_positions = self.move_group.get_current_joint_values()
        rotation_angle = radians(angle)
        current_joint_positions[joint_index - 1] += rotation_angle
        self.move_group.go(current_joint_positions)

    def force_callback(self, msg):
        """Callback to read force sensor values and set sign conventions."""
        self.force_x = msg.wrench.force.x
        self.force_y = msg.wrench.force.y 
        self.force_z = -msg.wrench.force.z

    def press(self, force_threshold=5.0, press_distance=0.003):
        """
        Press down until reaching the given force threshold or distance.
        Args:
            force_threshold (float): target force on Z [N]
            press_distance  (float): max travel on Z [m]
        """
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.3)

        rospy.loginfo("Pressing down...")

        rate = rospy.Rate(100)
        total_distance = 0.0
        step = 0.0005  # step per iteration

        while not rospy.is_shutdown():
            base_pose = self.move_group.get_current_pose().pose
            if self.force_z < force_threshold and total_distance < press_distance:
                delta_z = min(self.k_z * (force_threshold - self.force_z), step, press_distance - total_distance)
                target_pose = copy.deepcopy(base_pose)
                target_pose.position.z = base_pose.position.z - delta_z
                total_distance += delta_z

                rospy.loginfo(f"Pressing... Force Z: {self.force_z:.2f} N, Distance: {total_distance:.4f} m")

                (plan, _) = self.move_group.compute_cartesian_path([target_pose], 0.01)
                self.move_group.execute(plan, wait=True)
            else:
                rospy.loginfo("Desired pressure or distance reached. Stopping.")
                break

            rate.sleep()
        rospy.sleep(2)

    def press_force(self, force_threshold=5.0):
        """
        Press along Z until the measured force exceeds the threshold.
        Args:
            force_threshold (float): threshold [N]
        """
        init_force = self.force_z
        rospy.loginfo("Pressing down...")

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            base_pose = self.move_group.get_current_pose().pose
            force = self.force_z

            if force < force_threshold:
                target_pose = copy.deepcopy(base_pose)
                delta_z = min(self.k_z * (force_threshold - force), 0.001)
                target_pose.position.z = base_pose.position.z - delta_z

                rospy.loginfo(f"Pressing... Force Z: {force:.2f} N")

                (plan, _) = self.move_group.compute_cartesian_path([target_pose], 0.01)
                self.move_group.execute(plan, wait=True)
            else:
                rospy.loginfo("Force threshold reached. Stop pressing.")
                break

            rate.sleep()
            
    def push(self, force_threshold=7.0, direction=None, push_distance=0.05):
        """
        Push along a given axis until force threshold or distance is reached.
        Args:
            force_threshold (float): threshold [N]
            direction (list[int]): axis direction, one-hot among [x,y,z]
            push_distance (float): max distance [m]
        """
        if direction is None:
            direction = [0, 1, 0]  # default Y-axis

        rospy.loginfo("Pushing...")

        self.move_group.set_max_velocity_scaling_factor(0.4)
        self.move_group.set_max_acceleration_scaling_factor(0.4)
        
        force = 0.0
        rate = rospy.Rate(100)
        total_distance = 0.0

        while not rospy.is_shutdown():
            base_pose = self.move_group.get_current_pose().pose

            if direction == [1, 0, 0]:
                force = self.force_x
                rospy.loginfo("Pushing in X-axis.")
            elif direction == [0, 1, 0]:
                force = self.force_y
                rospy.loginfo("Pushing in Y-axis.")
            elif direction == [0, 0, 1]:
                force = self.force_z
                rospy.loginfo("Pushing in Z-axis.")
            else:
                rospy.logwarn("Invalid direction.")

            if force < force_threshold and total_distance < push_distance:
                target_pose = copy.deepcopy(base_pose)
                delta_position = [min(self.k * (force_threshold - force) * d, 0.04, push_distance - total_distance if d != 0 else 0) for d in direction]
                target_pose.position.x += delta_position[0]
                target_pose.position.y += delta_position[1]
                target_pose.position.z += delta_position[2]
                total_distance += sum([abs(dp) for dp in delta_position])

                rospy.loginfo(f"Pushing... Force : {force:.2f} N. Distance: {total_distance:.4f} m.")

                (plan, _) = self.move_group.compute_cartesian_path([target_pose], 0.01)
                self.move_group.execute(plan, wait=True)
            else:
                rospy.loginfo("Desired pressure or distance reached. Stopping push.")
                break

            rate.sleep()

    def rotate_joint_6(self, delta_degree):
        """
        Rotate joint-6 by a given degree amount.
        Args:
            delta_degree (float): rotation angle in degrees
        """
        joint_positions = self.move_group.get_current_joint_values()
        joint_positions[5] += radians(delta_degree)
        self.move_group.go(joint_positions, wait=True)
        self.move_group.stop()

    def gear_engagement(self, force_threshold=5.0, move_step=0.0005, max_down=0.007, rotate_degree=0.005):
        """
        Attempt gear engagement: press down; if collision force exceeded, rotate joint-6 slightly and retry.
        Args:
            force_threshold (float): Z-force delta threshold indicating contact
            move_step (float): downward step size [m]
            max_down (float): max downward travel [m]
            rotate_degree (float): rotation on joint-6 [deg] when contact detected
        """
        rospy.loginfo("Test engagement!")

        start_pose = self.move_group.get_current_pose().pose
        target_z = start_pose.position.z - max_down
        current_z = start_pose.position.z
        init_force_z = self.force_z
        print("Initial force_z:", init_force_z)
        step = 0.0
        while current_z > target_z and not rospy.is_shutdown():
            diff = self.force_z - init_force_z
            if diff < force_threshold:
                self.cartesian_relative_move('z', -move_step)
                rospy.sleep(0.05)
                current_z -= move_step
                step += move_step
                print(diff, "N")
            else:
                print(diff, "N")
                print("Reset")
                self.cartesian_relative_move('z', step)
                current_z += step
                self.rotate_joint_6(rotate_degree)
                step = 0.0
                rospy.sleep(0.5)

        rospy.loginfo("Gear engagement test completed")

    def check_grasp_success_by_position(self, target_position):
        """
        Heuristic grasp success check by reading gripper position.
        Returns:
            bool: True if likely grasped an object, False otherwise.
        """
        try:
            self.robotiq.sendall(b'GET POS\n')
            data = self.robotiq.recv(1024)
            # Expecting a response like: b'POS=xxx\n'
            pos_str = data.decode()
            pos_val = int(pos_str.split('=')[1])
            # If actual position deviates a lot from command, object might be in between
            if abs(pos_val - target_position) > 10:  # empirical threshold
                rospy.loginfo("Gripper grasp likely successful (by position).")
                return True
            else:
                rospy.logwarn("Gripper may not have grasped (by position).")
                return False
        except Exception as e:
            rospy.logwarn(f"Failed to read gripper position: {e}")
            return False

    def plan_and_execute(self, plan_name="gear_assembling"):
        """Wrapper to call skills.plan_execute.plan_and_execute."""
        plan_and_execute(self, plan_name)

    def load_plan(self, plan_name="all"):
        """Wrapper to call skills.plan_execute.load_plan."""
        return load_plan(self, plan_name)

    def desk_cleanup(self):
        # Poses used in the desk cleanup routine (kept as given)
        gripper_pose_1 = (0.29003609242900497,0.24138599199788655, 0.2845015784995799,0.6990476811778046, -0.7132429210313296, -0.03568994426570673, 0.03664836854406673 )
        paperclip_pose =( -0.11284821937505589,  0.4988046722288261, 0.3478929639143026, 0.9994973675328963, 0.0208084893348552,  -0.02390890554350973,  0.0006191139823840461)
        paperclip_box_pose = (0.022277337581302736, 0.4143429693088012, 0.4330746677311388, 0.9992086486833958, -0.028480992953358892, 0.027042274578199074, 0.00629482504365302)
        gripper_pose_2 = ( 0.29003609242900497,0.41738599199788655, 0.2857015784995799,0.6990476811778046, -0.7132429210313296, -0.03568994426570673, 0.03664836854406673)
        gripper_pose_3 = (-0.334195458313977, 0.25438599199788655, 0.2810798840507104, 0.6926060767737865, 0.719115813664144, 0.028730432986686625, 0.048413130187763456)
        gripper_pose_4 = (-0.337195458313977, 0.4319563965504313, 0.2808798840507104, 0.6926060767737865, 0.719115813664144, 0.028730432986686625, 0.048413130187763456)
        
        tape_pose = (0.08020998952326158, 0.44082903449919014, 0.2885338299507632, 0.9997773408649737, -0.003837991588368589, -0.006970075462950805, 0.01954370900239581)
        tape_box_pose = (0.1720279012293059, 0.379893420595374, 0.30480499698765406, -0.9990638991632087, 0.03824988278019675, 0.01249631245644593, 0.015878099102161897)
        cap_pose = (0.08069315786994348, 0.3858952990784292, 0.29274058915775156, 0.9998605953236932, -0.0053712076635465495, -0.006584279210497978, 0.014373145600837278)
        tape_box_pose_cap = (0.1729279012293059, 0.378493420595374, 0.30850499698765406, -0.9990638991632087, 0.03824988278019675, 0.01249631245644593, 0.015878099102161897)
        close_pensil_box = (0.09985239054277284, 0.4523936173919913, 0.24129792690343382, -0.9995709736137425, 0.026964264132019043, 0.004685957929536387, 0.010432591575255319)
        grasp_pensil_box = (0.10582207095071731, 0.5316359120470862, 0.21805196562068996, -0.7016560349425377, 0.7124792796448057, -0.006475406461132795, 0.003186505405346112)
        binder_clip = (0.1847797101596527, 0.29459526141194775, 0.2747210253294315, 0.9998016846377653, -0.01597893523647261, -0.004136812722504809, 0.011142342872758067)
        sticky_note = (0.0747797101596527, 0.26359526141194775, 0.28672210253294315, 0.9998016846377653, -0.01597893523647261, -0.004136812722504809, 0.011142342872758067)
        push_large_box = (-0.11427270599509906, 0.1616867550175647, 0.23282645131048152, 0.9999454627162566, -0.0076623164028748825, 0.006606627440033706, 0.0025909408295449804)
        move_tape_box_pose = (0.16158849197143568, 0.3869516315574512, 0.2606474921216017, 0.9999999338943185, 0.00020410766180665692, -0.00020411403504589363, 0.00022110830278149565)
        put_tape_box_pose = (-0.08048849197143568, 0.2569516315574512, 0.2606474921216017, 0.9999999338943185, 0.00020410766180665692, -0.00020411403504589363, 0.00022110830278149565)
        move_binder_clip_box = (0.1816145841777211, 0.22193064412031507, 0.22077543389392738, 0.9999999926493446, 7.71413184427539e-05, -5.1133848556022154e-05, 7.833171223733975e-05)
        put_binder_clip_box = (-0.15248849197143568, 0.2569516315574512, 0.2256474921216017, 0.9999999338943185, 0.00020410766180665692, -0.00020411403504589363, 0.00022110830278149565)

        switch_to_local_control()
        rospy.sleep(2)

        self.gripperMove(0, 8, 0)
        self.homing()

        # 1. Assemble tape box and cap
        self.gripperMove(0, 8, 0)

        self.homing()

        self.move_to_target_pose(gripper_pose_1)

        self.gripperMove(100, 8, 30)

        self.add_safe_pose(gripper_pose_1)

        self.move_to_target_pose(tape_pose)

        self.gripperMove(160, 8, 40)

        self.add_safe_pose(tape_pose)

        self.move_to_target_pose(tape_box_pose)

        switch_to_remote_control()
        rospy.sleep(2)

        impedance_control()

        switch_to_local_control()
        rospy.sleep(2)

        self.gripperMove(100, 8, 30)

        self.add_safe_pose(tape_box_pose)

        self.move_to_target_pose(gripper_pose_1)

        self.gripperMove(0, 8, 0)

        self.add_safe_pose(gripper_pose_1)

        self.homing()

        self.move_to_target_pose(gripper_pose_4)

        self.gripperMove(110, 8, 30)

        self.add_safe_pose(gripper_pose_4)

        self.homing()

        self.move_to_target_pose(cap_pose)

        self.gripperMove(170, 8, 50)

        self.add_safe_pose(cap_pose)

        self.move_to_target_pose(tape_box_pose_cap)
                
        switch_to_remote_control()
        rospy.sleep(2)

        impedance_control(deep=0.003)

        switch_to_local_control()
        rospy.sleep(2)

        self.press(force_threshold=7.0, press_distance=0.007)

        self.cartesian_relative_move('z', 0.002)

        self.gripperMove(100, 8, 30)

        self.add_safe_pose(tape_box_pose_cap)

        # 2. Put the sticky note back into the large box
        self.move_to_target_pose(sticky_note)

        self.gripperMove(170, 8, 120)

        self.cartesian_relative_move('z', 0.15)
        self.cartesian_relative_move('y', 0.01)
        self.cartesian_relative_move('x', -0.24)
        self.cartesian_relative_move('z', -0.13)

        self.gripperMove(100, 8, 20)

        self.cartesian_relative_move('z', 0.18)

        # 3. Put gripper_4 back to the gripper pose 4
        self.add_safe_pose(gripper_pose_4)

        self.move_to_target_pose(gripper_pose_4)

        self.gripperMove(0, 8, 0)

        self.add_safe_pose(gripper_pose_4)

        self.homing()

        # 4. Put tape box back in the large box
        self.add_safe_pose(gripper_pose_3)
        self.move_to_target_pose(gripper_pose_3)

        self.gripperMove(110, 8, 30)

        self.add_safe_pose(gripper_pose_3)

        self.homing()

        self.move_to_target_pose(move_tape_box_pose)

        self.gripperMove(220, 8, 200)

        self.add_safe_pose(move_tape_box_pose)

        self.move_to_target_pose(put_tape_box_pose)

        self.gripperMove(110, 8, 30)

        self.add_safe_pose(put_tape_box_pose)

        self.move_to_target_pose(gripper_pose_3)

        self.gripperMove(0, 8, 0)

        self.add_safe_pose(gripper_pose_3)

        self.homing()

        # 5. Put the binder clip back into the binder_clip box
        self.move_to_target_pose(gripper_pose_2)

        self.gripperMove(100, 8, 30)

        self.add_safe_pose(gripper_pose_2)

        self.move_to_target_pose(binder_clip)

        self.gripperMove(170, 8, 80)

        self.cartesian_relative_move('z', 0.035)
        self.cartesian_relative_move('y', -0.08)

        self.gripperMove(110, 8, 0)
        rospy.sleep(3)

        self.cartesian_relative_move('z', 0.1)
        self.cartesian_relative_move('y', -0.08)
        self.cartesian_relative_move('z', -0.1)
        self.push(force_threshold=3.0, direction=[0, 1, 0], push_distance=0.1)
        self.cartesian_relative_move('z', -0.005)
        self.press_force(force_threshold=9.0)

        self.safe_move()

        self.move_to_target_pose(gripper_pose_2)
        self.gripperMove(0, 8, 0)
        self.add_safe_pose(gripper_pose_2)

        # 6. Put the binder clip back to box the large box
        self.gripperMove(50, 8, 0)

        self.move_to_target_pose(move_binder_clip_box)

        self.gripperMove(170, 8, 100)

        self.add_safe_pose(move_binder_clip_box)

        self.move_to_target_pose(put_binder_clip_box)

        self.gripperMove(50, 8, 0)

        self.add_safe_pose(put_binder_clip_box)

        # 6. Close the large box
        self.move_to_target_pose(push_large_box)

        self.push(force_threshold=6.0, direction=[0, 1, 0], push_distance=0.155)

        self.cartesian_relative_move('y', -0.03)

        self.safe_move()

        self.homing()

    def gear_assembling(self):
        # Poses for gear assembling routine (kept as given)
        shaft_pose_1 = (0.14369431510187974, 0.2569207089935488, 0.3256725074597286, 0.9999999652499849, 1.6461387312695917e-06, -0.0001896613199867806, 0.0001831008000572563)
        hole_pose_1 = (-0.024276779551335066, 0.25309708615838502, 0.3724878843033385, 0.9999998491418547, -2.8630402395980427e-05, -0.00027074644704994766, 0.0004770670072073083)
        shaft_pose_2 = (0.13928184532002857, 0.3818883104357458, 0.36277929264402126, 0.9999999717825309, -0.00018019792642356104, -0.00012999638661795765, 8.405108142915141e-05)
        hole_pose_2 = (-0.14125655612077673, 0.36442239604764504, 0.379730161029172, 0.999999988968839, 4.391155633223172e-05, 6.309082044807018e-05, 0.0001270969919811964)
        gear_pose_3 = (-0.06739894659877901, 0.5168205249379753, 0.2706540851182947, 0.999998806436675, -2.118442396318147e-05, -0.0001700254130662976, 0.0015355024598802528)
        put_gear_pose_3 = (-0.025276821337661858, 0.2548839108421758, 0.3074614324318014, 0.9999998124456349, -9.147890717622178e-05, -0.0003168975475866121, 0.000516058377435521)
        gear_pose_1 = (-0.21128692269017157, 0.5249225166661387, 0.2715792870976222, 0.999999928396554, -5.057058107329426e-05, 2.6533962812367507e-05, 0.00037409283848134037)
        gear_pose_2 = (0.0966882951369765, 0.504865611791847, 0.3429448716377327, -0.9999999895005082, 2.4683576332476613e-05, -6.970069018618338e-05, 0.00012462551319395598)
        put_gear_pose_2 = (-0.14230295728220787, 0.2480562882191704, 0.3955460664134095, 0.9999999335845632, -3.686899185901448e-05, -3.161486149694329e-05, 0.0003612091459215146)
        put_gear_pose_1 = (-0.13824476081967432, 0.36700635110923104, 0.3266595856619401, 0.9999999851965284, 4.8129729512445874e-05, 6.661166550097954e-05, 0.0001511732714054253)
        gripper_pose_1 = (0.29003609242900497,0.24138599199788655, 0.2845015784995799,0.6990476811778046, -0.7132429210313296, -0.03568994426570673, 0.03664836854406673 )
        gripper_pose_2 = ( 0.29003609242900497,0.41738599199788655, 0.2857015784995799,0.6990476811778046, -0.7132429210313296, -0.03568994426570673, 0.03664836854406673)
        gripper_pose_3 = (-0.334195458313977, 0.25438599199788655, 0.2810798840507104, 0.6926060767737865, 0.719115813664144, 0.028730432986686625, 0.048413130187763456)
        gripper_pose_4 = (-0.337195458313977, 0.4319563965504313, 0.2808798840507104, 0.6926060767737865, 0.719115813664144, 0.028730432986686625, 0.048413130187763456)
        
        switch_to_local_control()
        rospy.sleep(2)

        controller.gripperMove(0, 8, 0)
        controller.homing()
        
        # 1. Assemble the shaft 1 into the hole
        self.move_to_target_pose(gripper_pose_1)
        self.gripperMove(100, 8, 20)
        self.add_safe_pose(gripper_pose_1)

        self.move_to_target_pose(shaft_pose_1)
        self.gripperMove(200, 8, 120)
        self.add_safe_pose(shaft_pose_1)

        self.move_to_target_pose(hole_pose_1)
        self.cartesian_relative_move('z', -0.03)
        self.gripperMove(100, 8, 20)
        self.add_safe_pose(hole_pose_1)

        # 2. Assemble the shaft 2 into the hole
        self.move_to_target_pose(shaft_pose_2)
        self.gripperMove(240, 8, 120)
        self.add_safe_pose(shaft_pose_2)
        self.move_to_target_pose(hole_pose_2)

        switch_to_remote_control()
        rospy.sleep(2)
        impedance_control()
        switch_to_local_control()
        rospy.sleep(2)

        self.cartesian_relative_move('z', -0.015)
        self.gripperMove(100, 8, 20)
        self.add_safe_pose(hole_pose_2)

        self.move_to_target_pose(gripper_pose_1)
        self.gripperMove(0, 8, 0)
        self.add_safe_pose(gripper_pose_1)

        # 3. Assemble the gear 1 to the shaft 1
        self.move_to_target_pose(gear_pose_1)
        self.gripperMove(250, 8, 120)
        self.add_safe_pose(gear_pose_1)
        self.move_to_target_pose(put_gear_pose_1)

        self.gripperMove(0, 8, 0)
        self.add_safe_pose(put_gear_pose_1)

        # 5. Assemble the gear 2 to the shaft 2
        self.move_to_target_pose(gripper_pose_4)
        self.gripperMove(100, 8, 30)
        self.add_safe_pose(gripper_pose_4)

        self.move_to_target_pose(gear_pose_2)
        self.gripperMove(250, 8, 120)
        self.add_safe_pose(gear_pose_2)
        self.move_to_target_pose(put_gear_pose_2)

        switch_to_remote_control()
        rospy.sleep(2)
        impedance_control()
        switch_to_local_control()
        rospy.sleep(2)
        self.cartesian_relative_move('z', -0.045)
        self.gear_engagement(force_threshold=0.8, move_step=0.0005, max_down=0.005, rotate_degree=2.5)
        self.cartesian_relative_move('z', -0.008)
        self.gripperMove(100, 8, 30)
        self.add_safe_pose(put_gear_pose_2)

        self.move_to_target_pose(gripper_pose_4)
        self.gripperMove(0, 8, 0)
        self.add_safe_pose(gripper_pose_4)

        # 4. Assemble the gear 3 to the shaft 3 
        self.move_to_target_pose(gear_pose_3)
        start_force_z = controller.force_z
        print("Start force_z:", start_force_z)
        self.gripperMove(250, 8, 130)
        self.add_safe_pose(gear_pose_3)

        self.move_to_target_pose(put_gear_pose_3)
        self.cartesian_relative_move('z', -0.02)
        self.gear_engagement(force_threshold=1.0, move_step=0.0005, max_down=0.005, rotate_degree=2.5)
        self.cartesian_relative_move('z', -0.008)
        self.gripperMove(0, 8, 0)
        self.add_safe_pose(put_gear_pose_3)
        self.homing()


if __name__ == "__main__":
    controller = UR5eController()

    if len(sys.argv) == 2:
        command = sys.argv[1]
        if command == "home":
            controller.homing()

        elif command == "rotate":
            controller.rotate_joint_6(10)

        elif command == "safe_move":
            controller.safe_move()

        elif command == "get_pose":
            controller.get_current_pose()

        elif command == "get_gripper":
            controller.gripperMove(100, 8, 30)

        elif command == "grasp":
            controller.gripperMove(220, 8, 120)

        elif command == "release":
            controller.gripperMove(0, 8, 0)

        elif command == "test_gripper":
            controller.gripperMove(100, 8, 20)
            # controller.gripperMove(150, 8, 60)

        elif command == "insertion":
            switch_to_remote_control()
            rospy.sleep(2)

            impedance_control()

            switch_to_local_control()
            rospy.sleep(2)

            controller.cartesian_relative_move('z', -0.02)
            controller.gripperMove(100, 8, 30)

        elif command == "engagement":
            controller.gear_engagement(force_threshold=1.0, move_step=0.0005, max_down=0.005, rotate_degree=2.5)

        elif command == "press":
            # controller.press(force_threshold=5.0, press_distance=0.008)
            controller.press_force(force_threshold=5.0)

        elif command == "push":
            controller.push(force_threshold=5.0, direction=[0, 1, 0], push_distance=0.08)
        
        elif command == "activate_gripper":
            # switch_to_local_control()
            controller.robotiq.sendall(b'SET ACT 1\n')

            controller.rotate_joint(-90, 6)

        elif command == "test":
            controller.homing()
            controller.gripperMove(0, 8, 0)
            controller.safe_move()
            controller.gripperMove(160, 8, 80)
            controller.homing()

        elif command == "desk_cleanup":
            controller.desk_cleanup()

        elif command == "gear_assembling":
            controller.gear_assembling()

        else:
            rospy.logwarn("Invalid command!")

    elif len(sys.argv) == 3:
        command , command_2 = sys.argv[1], sys.argv[2]

        if command == "cartesian_relative_move" and command_2 is not None:
            # Expect "axis,distance", e.g., "x,0.1"
            direction, distance = command_2.split(",")
            distance = float(distance)

            rospy.loginfo(f"Moving {distance} in {direction} direction")
            controller.cartesian_relative_move(direction, distance)
        elif command == "press":
            controller.press(command_2)

    else:
        rospy.logwarn("No command provided")

    # Clean shutdown
    controller.shutdown() 
