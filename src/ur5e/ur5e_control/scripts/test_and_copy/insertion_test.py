#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, TwistStamped
from scipy.signal import butter, lfilter
from moveit_commander import MoveGroupCommander, MoveItServo
from geometry_msgs.msg import WrenchStamped



class ForceFilter:
    def __init__(self):
        self.b, self.a = butter(2, 5, 'low', fs=500)  # 5Hz低通滤波
        self._prev_force = 0.0
        
    def filter(self, raw_force):
        filtered = 0.8*self._prev_force + 0.2*raw_force  # 简单指数滤波
        self._prev_force = filtered
        return filtered

class RobustInsertionController:
    def __init__(self):
        # 初始化MoveIt
        self.move_group = MoveGroupCommander("manipulator")
        self.servo = MoveItServo()
        self.servo.start()

        # 力传感器初始化
        self.force_sub = rospy.Subscriber("/wrench", WrenchStamped, self.force_callback)
        self.force_filter = ForceFilter()
        
        # 控制参数
        self.K_z = 500.0  # 刚度 (N/m)
        self.D_z = 30.0   # 阻尼 (Ns/m)
        self.Fz_threshold = 10.0  # 接触阈值(N)
        
        # 李萨如参数
        self.A_x = 0.005   # X振幅(m)
        self.A_y = 0.005   # Y振幅(m)
        self.freq_x = 0.5  # X频率(Hz)
        self.freq_y = 0.5  # Y频率(Hz)
        self.phase = np.pi/2  # 相位差

    def force_callback(self, msg):
        """带滤波的力传感器处理"""
        raw_force = msg.wrench.force.z
        self.force_z = self.force_filter.filter(raw_force)

    def insertion(self):
        rate = rospy.Rate(500)
        base_pose = self.move_group.get_current_pose().pose
        start_z = base_pose.position.z
        target_depth = 0.01  # 目标插入深度5cm
        
        # 初始化控制变量
        last_time = rospy.Time.now().to_sec()
        integral_error = 0.0
        last_error = 0.0
        
        while not rospy.is_shutdown():
            # 计算时间步长
            current_time = rospy.Time.now().to_sec()
            dt = current_time - last_time
            last_time = current_time
            
            # 生成李萨如轨迹
            t = current_time - start_time
            dx = self.A_x * np.sin(2*np.pi*self.freq_x*t + self.phase)
            dy = self.A_y * np.sin(2*np.pi*self.freq_y*t)
            
            # 阻抗控制计算
            force_error = self.force_z - self.Fz_threshold
            derivative_error = (force_error - last_error) / dt
            
            # PID控制（示例使用PD）
            delta_z = (force_error/self.K_z) - (derivative_error*self.D_z)
            delta_z = np.clip(delta_z, -0.005, 0.005)  # 限制最大偏移
            
            # 构建目标位姿
            target_pose = Pose()
            target_pose.position.x = base_pose.position.x + dx
            target_pose.position.y = base_pose.position.y + dy
            target_pose.position.z = start_z - insertion_depth + delta_z
            target_pose.orientation = base_pose.orientation
            
            # 通过Servo发送速度指令
            twist_msg = TwistStamped()
            twist_msg.twist.linear.x = 2*np.pi*self.freq_x*self.A_x*np.cos(2*np.pi*self.freq_x*t + self.phase)
            twist_msg.twist.linear.y = 2*np.pi*self.freq_y*self.A_y*np.cos(2*np.pi*self.freq_y*t)
            twist_msg.twist.linear.z = insertion_speed + delta_z/dt if dt>0 else 0
            self.servo.publish_twist(twist_msg)
            
            # 插入深度监控
            insertion_depth = start_z - self.move_group.get_current_pose().pose.position.z
            if insertion_depth >= target_depth:
                rospy.loginfo("Insertion completed with depth: %.4f m", insertion_depth)
                break
                
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('robust_insertion_controller')
    controller = RobustInsertionController()
    controller.insertion()