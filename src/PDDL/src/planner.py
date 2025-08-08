# src/planner.py
import os
from pyperplan.pddl import parser
from pyperplan import grounding, search 

def solve(domain_file, problem_file):
    par = parser.Parser(domain_file) 
    # 1. 解析 PDDL
    dom = parser.Parser().parse_domain(domain_file)
    prob = parser.Parser().parse_problem(problem_file, dom)
    # 2. ground（生成可搜索任务）
    task = grounding.ground(prob)
    # 3. 搜索（A* + 默认启发式）
    plan = search.get_plan(task)
    return plan  # List[(Action, [参数...])]

def to_planlist(raw_plan):
    planlist = []
    for action, args in raw_plan:
        name = action.name
        if name.startswith("goto"):
            # 将最后一个参数作为目标位置
            planlist.append([f"goto,{args[-1]}"])
        else:
            planlist.append([name])
    return planlist

# def solve(domain_file, problem_file):
#     # 1. 解析 PDDL（新版 API）
#     dom = parse_pddl_file("domain", domain_file)  # ✅ 直接解析 domain
#     prob = parse_pddl_file("problem", problem_file)  # ✅ 直接解析 problem
#     # 2. ground（生成可搜索任务）
#     task = grounding.ground(prob)
#     # 3. 搜索（A* + 默认启发式）
#     plan = search.get_plan(task)
#     return plan

if __name__ == "__main__":
    here = os.path.dirname(__file__)
    domain_pddl  = os.path.join(here, "../domain.pddl")
    problem_pddl = os.path.join(here, "../problem.pddl")
    raw_plan = solve(domain_pddl, problem_pddl)
    if raw_plan is None:
        print("没有找到可行计划")
    else:
        print("Raw plan:", raw_plan)
        print("Planlist:", to_planlist(raw_plan))
