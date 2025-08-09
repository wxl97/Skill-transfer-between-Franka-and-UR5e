import numpy as np
import time

# from rtde_control import RTDEControlInterface
# from rtde_receive import RTDEReceiveInterface
from math import pi, sin

def impedance_control(deep=0.01):

    # 机器人IP
    robot_ip = "172.16.15.10"  
    rtde_c = RTDEControlInterface(robot_ip)
    rtde_r = RTDEReceiveInterface(robot_ip)

    # 目标位姿 [x, y, z, rx, ry, rz]
    # target_pose = [0.3, 0.0, 0.4, 0.0, 3.14, 0.0]

    # 获取当前姿态作为目标
    target_pose = rtde_r.getActualTCPPose()

    # 相关参数:刚度、阻尼和积分项数组
    Kp_pos = np.array([200.0, 200.0, 28.0])
    Kd_pos = np.array([10.0, 10.0, 1.5])
    Ki_pos = np.array([30.0, 30.0, 10.0])

    Kp_ori = np.array([100.0, 100.0, 80.0])
    Kd_ori = np.array([10.0, 10.0, 8.0])
    Ki_ori = np.array([20.0, 20.0, 18.0])

    # 积分项初始化
    integral_error = np.zeros(3)  
    integral_error_ori = np.zeros(3) 

    # 李萨如轨迹参数
    A_x = 0.003
    A_y = 0.003
    freq_x = 1.5
    freq_y = 1.0
    sphase = np.pi / 2.0

    #Test parameter
    # A_x = 0.1
    # A_y = 0.1
    # freq_x = 0.5
    # freq_y = 0.3
    # sphase = pi / 2.0

    # 目标位姿的初始值
    base_x = target_pose[0]
    base_y = target_pose[1]
    base_z = target_pose[2]


    # 控制周期
    dt = 0.002  # 500Hz


    # 力控参数配置
    task_frame = [0, 0, 0, 0, 0, 0]         # 力控基准点，基坐标系
    selection_vector = [1, 1, 1, 0, 0, 0]    # 全六自由度阻抗
    force_mode_type = 2                     # 固定参考系
    speed_limits = [1.0, 1.0, 1.0, 0.5, 0.5, 0.5]  # 限制最大速度

    try:
        prev_pose = np.array(rtde_r.getActualTCPPose())
        current_pose = rtde_r.getActualTCPPose()
        time.sleep(dt)

        # 开启力控模式
        rtde_c.forceMode(task_frame, selection_vector, [0,0,0,0,0,0],
                        force_mode_type, speed_limits)
        
        t0 = time.time()

        while current_pose[2] > base_z - deep :

            

            current_pose = rtde_r.getActualTCPPose()

            # 假设z值保持不变，或者根据需求调整
            x = base_x + A_x * sin(2 * pi * freq_x * (time.time() - t0) + sphase)
            y = base_y + A_y * sin(2 * pi * freq_y * (time.time() - t0))
            if time.time() - t0 > 10.0 and time.time() - t0 < 11.0: 
                middle_pose = rtde_r.getActualTCPPose()
                z = middle_pose[2] + 0.003  # 保持z值不变
            elif time.time() - t0 <= 10.0:
                z = base_z - 0.01 - deep 
            else:
                t0 = time.time()
                z = base_z - 0.01 - deep 
                

            # 生成目标位姿
            target_pose[0] = x
            target_pose[1] = y
            target_pose[2] = z

            # 当前位姿
            current_pose_matrix = np.array(current_pose)

            # 读取速度
            velocity = np.array(rtde_r.getActualTCPSpeed())

            # 位置误差
            pos_error = target_pose[:3] - current_pose_matrix[:3]
            ori_error = target_pose[3:] - current_pose_matrix[3:]

            # 位置误差积分
            integral_error += pos_error * dt
            integral_error_ori += ori_error * dt

            # 限制积分误差
            integral_error = np.clip(integral_error, -0.1, 0.1)
            integral_error_ori = np.clip(integral_error_ori, -0.1, 0.1)

            # 期望力 = 刚度项 + 阻尼项 + 积分项
            force = Kp_pos * pos_error + Kd_pos * velocity[:3] + Ki_pos * integral_error
            torque = Kp_ori * ori_error + Kd_ori * velocity[3:] + Ki_ori * integral_error_ori

            wrench = np.concatenate((force, torque)).tolist()

            # forceMode命令
            rtde_c.forceMode(task_frame, selection_vector, wrench,
                            force_mode_type, speed_limits)

            # 更新prev
            prev_pose = current_pose_matrix.copy()

            time.sleep(0.002)  # 保持500Hz


    finally:
        # 无论如何最后要停掉force mode
        rtde_c.forceModeStop()
        rtde_c.stopScript()

# if __name__ == '__main__':
#     impedance_control()
    
    