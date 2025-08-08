#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
import roslaunch
import os, sys
import rospkg  


rospack = rospkg.RosPack()
PANDA_PKG_PATH = rospack.get_path('panda_base')
FRANKA_LAUNCH = os.path.join(PANDA_PKG_PATH,
                             'launch',
                             'panda_with_impedance_controller.launch')

# # 动态添加 catkin_ws 的路径到 sys.path（保持不变）
# sys.path.append(os.path.join(os.getenv('HOME'), '/../../../franka_ws/src/panda_base/scripts'))


# 导入控制类
try:
    from panda_control import Franka_control
except ImportError:
    Franka_control = None

class RobotInterface:
    """
    统一机器人接口：
      1) 启动 FRANKA 节点；
      2) 动态实例化控制器；
      3) 普通动作 execute(action, *args)；
      4) 专用 execute_plan(task_name)；
    """
    def __init__(self, robot_type, **kwargs):
        # 1) 初始化 ROS
        if not rospy.core.is_initialized():
            rospy.init_node('robot_interface_node', anonymous=True)

        # 2) 启动底层控制器 launch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        if robot_type.lower() == 'franka':
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, [FRANKA_LAUNCH])
        else:
            raise ValueError(f"不支持的 robot_type: {robot_type}")
        rospy.loginfo("[Interface] 启动控制器节点...")
        self.launch.start()
        rospy.sleep(2)

        # 3) 实例化控制器
        self.robot_type = robot_type.lower()
        if self.robot_type == 'franka':
            if Franka_control is None:
                raise ImportError("未找到 Franka_control")
            rospy.loginfo("[Interface] 初始化 Franka 控制器...")
            self.robot = Franka_control(**kwargs)
        else:
            raise ValueError(f"不支持的 robot_type: {robot_type}")

        # —— 新增：加载 PDDL planlist.json —— 
        planlist_path = os.path.join(os.path.dirname(__file__), '..', 'PDDL', 'planlist.json')
        try:
            with open(planlist_path, 'r', encoding='utf-8') as f:
                self.planlist = json.load(f)
            rospy.loginfo(f"[Interface] 已加载 planlist.json，包含任务：{list(self.planlist.keys())}")
        except Exception as e:
            rospy.logerr(f"[Interface] 加载 planlist.json 失败: {e}")
            self.planlist = {}

        # —— 新增：加载全局知识库 —— 
        kb_path = os.path.join(os.path.dirname(__file__), '..', 'global_know_ledge_base.json')
        try:
            with open(kb_path, 'r', encoding='utf-8') as f:
                self.kb = json.load(f)
            rospy.loginfo(f"[Interface] 已加载 global_know_ledge_base.json，机器人条目：{list(self.kb.keys())}")
        except Exception as e:
            rospy.logerr(f"[Interface] 加载 global_know_ledge_base.json 失败: {e}")
            self.kb = {}

    def execute(self, action, *args, **kwargs):
        """
        通用执行：调用底层控制器的方法。
        action: 方法名；args/kwargs: 传给方法的参数
        """
        rospy.loginfo(f"[Interface] 执行动作: {action}，参数: {args} {kwargs}")
        if not hasattr(self.robot, action):
            raise AttributeError(f"机器人不支持动作: {action}")
        method = getattr(self.robot, action)
        result = method(*args, **kwargs)
        if result is False:
            rospy.logwarn(f"动作 {action} 未成功完成")
            raise RuntimeError(f"动作 {action} 执行失败")
        rospy.loginfo(f"动作 {action} 已成功完成")
        return result

    def can_execute_task(self, task_name):
        """
        检查当前机器人是否具备执行 self.planlist[task_name] 所有技能。
        """
        if task_name not in self.planlist:
            rospy.logwarn(f"[Interface] planlist 中不存在任务: {task_name}")
            return False
        supported = self.kb.get(self.robot_type, [])
        for step in self.planlist[task_name]:
            action = step[0]
            if action not in supported:
                rospy.logwarn(f"[Interface] 机器人 '{self.robot_type}' 不支持技能: {action}")
                return False
        return True

    def execute_plan(self, task_name):
        """
        按顺序读取 planlist[task_name]，执行每一步：
          - 对于 go_to，step = ["go_to", "pose_name"]，会传入 pose_name；
          - 其他动作均不带参数，从知识库或内部读取必要信息。
        """
        if not self.can_execute_task(task_name):
            raise RuntimeError(f"能力检查未通过，无法执行任务 {task_name}")
        rospy.loginfo(f"[Interface] 开始执行任务序列：{task_name}")
        for step in self.planlist[task_name]:
            action = step[0]
            args   = step[1:]  # go_to 会有一个 pose_name，其它为空
            rospy.loginfo(f"[Interface] ⇒ 执行 {action}，args={args}")
            self.execute(action, *args)
        rospy.loginfo(f"[Interface] 任务 {task_name} 执行完毕")

    def shutdown(self):
        """
        关闭启动的 roslaunch 节点
        """
        rospy.loginfo("[Interface] 关闭控制器节点...")
        self.launch.shutdown()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("用法: python interface.py <robot_type> <action|execute_plan> [task_name 或 action 参数]")
        sys.exit(1)

    robot_type = sys.argv[1]
    action     = sys.argv[2]
    raw_args   = sys.argv[3:]

    # 分离 pos_args 和 --key val
    pos, kw = [], {}
    i = 0
    while i < len(raw_args):
        if raw_args[i].startswith('--'):
            k = raw_args[i][2:]
            v = raw_args[i+1]
            try:    v = int(v)
            except: 
                try: v = float(v)
                except: pass
            kw[k] = v
            i += 2
        else:
            v = raw_args[i]
            try:    v = int(v)
            except:
                try: v = float(v)
                except: pass
            pos.append(v)
            i += 1

    iface = RobotInterface(robot_type)
    try:
        if action == 'execute_plan' and pos:
            # 示例： python interface.py franka execute_plan shaft1
            iface.execute_plan(pos[0])
        else:
            # 普通单步调用
            iface.execute(action, *pos, **kw)
    except Exception as e:
        rospy.logerr(f"执行失败: {e}")
    finally:
        iface.shutdown()
