import rospy

def get_current_joint_positions(move_group):
    '''读取并在命令行输出当前的关节坐标。'''
    current_joint_positions = move_group.get_current_joint_values()
    positions_str = ", ".join(["%.6f" % pos for pos in current_joint_positions])
    rospy.loginfo("当前关节坐标: %s", positions_str)
    return current_joint_positions

def read_current_pose(move_group):
    ''' Read and print the current pose (quaternion) and gripper width.'''
    current_pose = move_group.get_current_pose().pose
    key_name = input("Please enter the name for the current pose: ")
    import json
    pose_data = {
        key_name: {
            "Goal": (
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z,
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w
            )
        }
    }
    filename = 'Object_Pose.json'
    try:
        with open(filename, 'w') as json_file:
            json.dump(pose_data, json_file, indent=4)
        rospy.loginfo("Current Pose: (%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f)" %
                      (current_pose.position.x, current_pose.position.y, current_pose.position.z,
                       current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                       current_pose.orientation.w))
        rospy.loginfo("Pose data saved to %s" % filename)
    except Exception as e:
        rospy.logerr("Failed to save pose data: %s" % str(e))
    return current_pose
