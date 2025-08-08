#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import rospy
import roslaunch


# 动态添加 catkin_ws 的路径到 sys.path
sys.path.append(os.path.join(os.getenv('HOME'), 'xinlong/catkin_ws/src/panda_base/scripts'))


# Launch file paths (按实际路径修改)
FRANKA_LAUNCH = os.path.join(
    os.getenv('HOME'), '/home/blackbird/xinlong/catkin_ws/src/panda_base/launch/panda_with_impedance_controller.launch'
)
# UR5_LAUNCH = os.path.join(
#     os.getenv('HOME'), 'catkin_ws/src/ur5_control_pkg/launch/ur5_control.launch'
# )

# 导入各个机器人的控制类（在各自包中仅在 __main__ 初始化节点）
try:
    from panda_control import Franka_control
except ImportError:
    Franka_control = None

# try:
    #     from ur5_control.scripts.ur5_controller import UR5eController
    # except ImportError:
#         UR5eController = None


class RobotInterface:
    """
    统一机器人接口：
    1. 启动 FRANKA 和 UR5e 的 roslaunch 节点；
    2. 根据 robot_type 动态实例化对应控制器；
    3. 提供 execute(action, ...) 方法统一调度。

    支持 robot_type: 'franka', 'ur5e'
    """

    def __init__(self, robot_type, **kwargs):
        # 1) 初始化 ROS 节点
        if not rospy.core.is_initialized():
            rospy.init_node('robot_interface_node', anonymous=True)

        # 2) 启动底层控制器节点（仅一次）
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        if robot_type.lower() == 'franka':
            self.launch = roslaunch.parent.ROSLaunchParent(
                uuid,
                [FRANKA_LAUNCH]
            )
        elif robot_type.lower() == 'ur5e':
            # self.launch = roslaunch.parent.ROSLaunchParent(
            #     uuid,
            #     [
            #         # UR5_LAUNCH
            #     ]
            # )
            raise ValueError("UR5e 部分暂时被注释，无法启动")
        else:
            raise ValueError(f"不支持的 robot_type: {robot_type}")

        rospy.loginfo("[Interface] 启动控制器节点...")
        self.launch.start()
        rospy.sleep(2)  # 等待各节点初始化

        # 3) 动态实例化机器人控制器
        self.robot_type = robot_type.lower()
        if self.robot_type == 'franka':
            if Franka_control is None:
                raise ImportError("未找到 Franka_control")
            rospy.loginfo("[Interface] 初始化 Franka 控制器...")
            self.robot = Franka_control(**kwargs)
        elif self.robot_type in ('ur5', 'ur5e'):
            # if UR5eController is None:
            #     raise ImportError("未找到 UR5eController，检查包名及路径。")
            # rospy.loginfo("[Interface] 初始化 UR5e 控制器...")
            # self.robot = UR5eController(**kwargs)
            raise ValueError("UR5e 部分暂时被注释，无法初始化")
        else:
            raise ValueError(f"不支持的 robot_type: {robot_type}")

    def execute(self, action, *args, **kwargs):
        """
        通用执行：调用底层控制器的方法。
        action: 方法名；args/kwargs: 传给方法的参数
        """
        rospy.loginfo(f"[Interface] 执行动作: {action}，参数: {args}, {kwargs}")
        if not hasattr(self.robot, action):
            raise AttributeError(f"机器人不支持动作: {action}")
        method = getattr(self.robot, action)
        result = method(*args, **kwargs)
        if result is False:
            rospy.logwarn(f"动作 {action} 未成功完成")
            raise RuntimeError(f"动作 {action} 执行失败")
        rospy.loginfo(f"动作 {action} 已成功完成")
        return result

    def shutdown(self):
        """
        关闭启动的 roslaunch 节点
        """
        rospy.loginfo("[Interface] 关闭控制器节点...")
        self.launch.shutdown()


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 3:
        print("用法: python interface.py <robot_type> <action> [pos_args...] [--key val...]")
        sys.exit(1)

    robot_type = sys.argv[1]
    action = sys.argv[2]
    raw = sys.argv[3:]
    pos, kw = [], {}
    i = 0
    while i < len(raw):
        if raw[i].startswith('--'):
            k = raw[i][2:]
            v = raw[i+1]
            try:
                v = int(v)
            except:
                try:
                    v = float(v)
                except:
                    pass
            kw[k] = v
            i += 2
        else:
            v = raw[i]
            try:
                v = int(v)
            except:
                try:
                    v = float(v)
                except:
                    pass
            pos.append(v)
            i += 1

    iface = RobotInterface(robot_type)
    try:
        res = iface.execute(action, *pos, **kw)
        if res is True:
            rospy.loginfo(f"[{robot_type}] 动作 {action} 成功")
        else:
            rospy.logwarn(f"[{robot_type}] 动作 {action} 返回: {res}")

        # # 示例：如需批量调用多个技能，可参考如下写法（不影响原有命令行功能）
        # iface.execute('homing')
        # iface.execute('grasp')
        # iface.execute('move_down', 0.05)
        # joints = iface.execute('get_current_joint_positions')
        # print("当前关节角:", joints)
        # pose = iface.execute('read_current_pose')
        # print("当前末端位姿:", pose)

        # # 新增代码示例
        # iface = RobotInterface('franka')
        # iface.execute('plan_and_execute', plan_name='gear1')
        # iface.shutdown()

    except Exception as e:
        rospy.logerr(f"执行失败: {e}")
    finally:
        iface.shutdown()
