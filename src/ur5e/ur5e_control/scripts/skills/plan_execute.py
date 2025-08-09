import copy
from skills.plan import Plan, PlanEl
import os
import json


def plan_and_execute(self, plan_name="all"):
    """
    Plan and execute a pick-and-place operation.
    """
    self.known_poses = copy.deepcopy(self.known_poses_or)
    self.add_top_poses()

    plan_list = self.load_plan(plan_name)
    plan = Plan(plan_list, self)
    plan.execute()
    self.n_executions += 1

def load_plan(self, plan_name="all"):
    config_path = "../plan_config.json"
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Plan config file not found: {config_path}")
    
    with open(config_path, "r") as f:
        plan_config = json.load(f)
    
    plan_raw = plan_config.get(plan_name, [])
    plan_list = []
    for item in plan_raw:
        if len(item) == 2:
            plan_list.append(PlanEl(item[0], item[1]))
        elif len(item) == 1:
            plan_list.append(PlanEl(item[0]))
    return plan_list