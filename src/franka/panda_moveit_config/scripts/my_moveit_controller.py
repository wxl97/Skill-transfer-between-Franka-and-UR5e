#!/usr/bin/env python3

import rospy
import sys

import moveit_commander
import moveit_msgs.msg

from std_msgs.msg import String

def controller():
    rospy.init_node('my_moveit_controller', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group = moveit_commander.MoveGroupCommander("panda_arm")
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    
    # rospy.Subscriber("chatter", String, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass


