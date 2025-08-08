#!/usr/bin/env python3
# soln2planlist.py

import re
import json
import sys
import os

# ----------- 自定义映射规则 -------------
ACTION_RULES = {
    'goto_pickup':    lambda args: ["go_to", args[-1]],
    'goto_place':     lambda args: ["go_to", args[-1]],
    'grasp':                lambda args: ["grasp"],
    'grasp_without_gripper':lambda args: ["grasp_without_gripper"],
    'insertion':            lambda args: ["insertion"],
    'release_gripper':      lambda args: ["release_gripper"],
    'change_gripper':       lambda args: ["change_gripper"],
    # 如果将来有更多动作，继续在这里加...
}

def default_rule(action, args):
    """没有特别规则时，原样输出 action+args"""
    return [action] + args

# ----------- 解析 soln ---------
def parse_solution(soln_path):
    planlist = []
    pattern = re.compile(r'^\s*\(\s*(\S+)(?:\s+(.*?))?\s*\)\s*$')
    with open(soln_path, 'r', encoding='utf-8') as f:
        for line in f:
            m = pattern.match(line.strip())
            if not m:
                continue
            action = m.group(1)
            args   = m.group(2).split() if m.group(2) else []
            if action in ACTION_RULES:
                planlist.append(ACTION_RULES[action](args))
            else:
                planlist.append(default_rule(action, args))
    return planlist

# ----------- 主程序 -------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: soln2planlist.py <solution.soln> [> planlist.json]")
        sys.exit(1)

    soln_file = sys.argv[1]
    if not os.path.isfile(soln_file):
        print(f"文件不存在：{soln_file}")
        sys.exit(1)

    # 从文件名提取 task name
    base = os.path.basename(soln_file)
    # 去掉最后一个扩展名，比如 problem.pddl.soln -> problem.pddl
    name1, _ = os.path.splitext(base)
    # 再去一次扩展名 problem.pddl -> problem
    task_name, _ = os.path.splitext(name1)

    # 解析 plan
    plan = parse_solution(soln_file)
    plans = { task_name: plan }

    # 先 dump 带缩进
    json_str = json.dumps(plans, ensure_ascii=False, indent=2)

    # 再把所有纯单元素列表压缩成一行：["xxx"]
    # 匹配形如:
    # [
    #   "some_action"
    # ]
    # 替成 ["some_action"]
    json_str = re.sub(
        r'\[\n\s+"([^"]+)"\n\s+\]',
        r'["\1"]',
        json_str
    )

    print(json_str)
