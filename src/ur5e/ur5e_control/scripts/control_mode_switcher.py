#!/usr/bin/env python3
import rospy
from ur_dashboard_msgs.srv import Load
from std_srvs.srv import Trigger
from ur_msgs.srv import SetSpeedSliderFraction
import subprocess

def switch_to_local_control(doc='/programs/xinlong.urp', speed=0.5):
    
    rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program')
    rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
    rospy.wait_for_service('/ur_hardware_interface/set_speed_slider')
    
    try:
        # 加载 URP 程序
        load = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        resp1 = load(doc)  # 这里替换为你实际的 URP 路径
        rospy.loginfo(f"Load response: {resp1.success}, {resp1.answer}")

        # 播放程序（进入 Local Control）
        play = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        resp2 = play()
        rospy.loginfo(f"Play response: {resp2.success}, {resp2.message}")

                # 设置速度
        set_speed = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)
        resp3 = set_speed(speed)
        rospy.loginfo(f"[Local] Set speed: {resp3.success}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def switch_to_remote_control():
    rospy.wait_for_service('/ur_hardware_interface/dashboard/stop')
    try:
        stop = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        resp = stop()
        rospy.loginfo(f"Stop response: {resp.success}, {resp.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Stop failed: {e}")

if __name__ == '__main__':
    rospy.init_node('control_mode_switcher')
    
    switch_to_local_control()

#     rospy.sleep(rospy.get_param("~local_duration", 25.0)Wx)
#     command = "bash -c 'source ~/ur5e_ws/devel/setup.bash && rosrun ur5e_control ur5e_control.py test'"
#     subprocess.run(command, shell=True)
#     rospy.sleep(rospy.get_param("~local_duration", 10.0))

#     switch_to_remote_control()
#     subprocess.run(["/bin/python3", "/home/jingyun/ur5e_ws/src/ur5e_control/scripts/ur5e_impedance_controller.py"])
#     rospy.sleep(rospy.get_param("~local_duration", 10.0))

    # # 切换模式
    # mode = rospy.get_param('~mode', 'local')  # 可设置为 remote 或 local
    # if mode == 'local':
    #     switch_to_local_control()
    # elif mode == 'remote':
    #     switch_to_remote_control()
    # else:
    #     rospy.logwarn("Unknown mode, please use 'local' or 'remote'")

