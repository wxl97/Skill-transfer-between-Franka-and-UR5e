#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import copy
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped,Pose
from math import radians, pi, sin



rospy.init_node('pose_stamped_publisher', anonymous=True)
pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)

cartesian_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
rate = rospy.Rate(10)  # 10 Hz

rospy.sleep(1)

current_pose = Pose()

# 李萨如参数
A_x = 0.05
A_y = 0.05
freq_x = 1.0
freq_y = 1.0
phase = pi / 2.0




def normalize_quaternion(q):
    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm
    return q


def homing():
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "panda_link0"
    msg.pose.position.x = 0.368511
    msg.pose.position.y = -0.009129
    msg.pose.position.z = 0.531829
    
    msg.pose.orientation.x = -0.922423
    msg.pose.orientation.y = 0.385891
    msg.pose.orientation.z = -0.013270
    msg.pose.orientation.w = 0.006947
    # msg.pose.orientation.x = current_pose.orientation.x
    # msg.pose.orientation.y = current_pose.orientation.y
    # msg.pose.orientation.z = current_pose.orientation.z
    # msg.pose.orientation.w = current_pose.orientation.w
    # msg.pose.orientation = normalize_quaternion(msg.pose.orientation)

    # 持续发布 1 秒确保机器人收到指令
    start_time = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - start_time) < 1.0:
        pub.publish(msg)
        rospy.loginfo("Homing to initial pose...")
        rospy.sleep(0.1)





def franka_state_callback(msg):
    global current_pose
    # 从 FrankaState 消息中提取末端执行器的位姿
    # O_T_EE 是一个 4x4 的变换矩阵，表示末端执行器相对于基座坐标系的位置和姿态
    current_pose.position.x = msg.O_T_EE[12]  # X 位置
    current_pose.position.y = msg.O_T_EE[13]  # Y 位置
    current_pose.position.z = msg.O_T_EE[14]  # Z 位置

    current_pose.orientation.x = msg.O_T_EE[0]  # 四元数 X
    current_pose.orientation.y = msg.O_T_EE[1]  # 四元数 Y
    current_pose.orientation.z = msg.O_T_EE[2]  # 四元数 Z
    current_pose.orientation.w = msg.O_T_EE[3]  # 四元数 W

    # 打印当前位姿
    rospy.loginfo("Current Pose: \n%s", current_pose)


def Lissajous():

    rate = rospy.Rate(10)
    t = 0.0
    
    initial_pose = copy.deepcopy(current_pose)

    while not rospy.is_shutdown():
       # 创建一个 PoseStamped 消息对象
        msg = PoseStamped()

        # 填充 header 部分：时间戳和参考坐标系
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"  

        msg.pose.position.x = initial_pose.position.x + A_x * sin(2 * pi * freq_x * t + phase)
        msg.pose.position.y = initial_pose.position.y + A_y * sin(2 * pi * freq_y * t)
        msg.pose.position.z = initial_pose.position.z
        # msg.pose.position.z = initial_pose.position.z - t * 0.005

        # msg.pose.orientation.x = current_pose.orientation.x
        # msg.pose.orientation.y = current_pose.orientation.y
        # msg.pose.orientation.z = current_pose.orientation.z
        # msg.pose.orientation.w = current_pose.orientation.w

        # msg.pose.orientation.x = current_pose.orientation.x
        # msg.pose.orientation.y = current_pose.orientation.y
        # msg.pose.orientation.z = current_pose.orientation.z
        # msg.pose.orientation.w = current_pose.orientation.w

        msg.pose.orientation.x = initial_pose.orientation.x
        msg.pose.orientation.y = initial_pose.orientation.y
        msg.pose.orientation.z = initial_pose.orientation.z
        msg.pose.orientation.w = initial_pose.orientation.w
        msg.pose.orientation = normalize_quaternion(msg.pose.orientation)

        # 发布消息
        pub.publish(msg)

        # rospy.loginfo("Publishing PoseStamped message: %s", msg)

        t += 1.0 / 10.0

        rate.sleep()

def cartesian_move(pose):
    # 创建 PoseStamped 消息
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = 'panda_link0'  # 设置坐标系为 'panda_link0'
    pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pose[0], pose[1], pose[2]
    pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = pose[3], pose[4], pose[5], pose[6]

    pose_msg.pose.orientation = normalize_quaternion(pose_msg.pose.orientation)
    # 发布消息
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing pose: %s", pose_msg)
        cartesian_pose_pub.publish(pose_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, franka_state_callback)

        homing()

        rospy.sleep(1)

        Lissajous()
        # pose = [0.368511, -0.009129, 0.531829, -0.922423, 0.385891, -0.013270, 0.006947]
        # cartesian_move(pose)
    except rospy.ROSInterruptException:
        pass
