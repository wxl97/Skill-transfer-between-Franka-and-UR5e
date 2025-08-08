import rospy

def gear_engagement(move_group, force_get_z, force_z, move_down, rotate_angle):
    rospy.loginfo("Test engagement!")
    move_group.set_max_velocity_scaling_factor(0.3)
    move_group.set_max_acceleration_scaling_factor(0.05)
    current_cartesian_pose = move_group.get_current_pose().pose
    target_force = force_get_z + 0.03
    target_z = current_cartesian_pose.position.z - 0.007
    while(current_cartesian_pose.position.z > target_z):
        step = 0.0
        while(force_get_z < target_force):
            move_down(0.0005)
            step -= 0.0005
            if(step < -0.003):
                return
        step -= 0.001
        move_down(step)
        rotate_angle(2.0, duration=3.0, acc=0.0)
        rospy.loginfo("External z force: %s", force_z)
        current_cartesian_pose = move_group.get_current_pose().pose
