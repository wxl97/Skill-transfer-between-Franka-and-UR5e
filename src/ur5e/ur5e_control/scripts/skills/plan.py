class Plan():
    def __init__(self, plan_elements, move_group_obj):
        self.plan_elements = plan_elements
        PlanEl.move_group_obj = move_group_obj

    def remove(self, pose_name):
        self.plan_elements = [plan_el for plan_el in self.plan_elements if plan_el.pose_name != pose_name]

    def execute(self):
        for plan_el in self.plan_elements:
            plan_el.execute()

class PlanEl():
    def __init__(self, pose_name, gripper_action=None):
        self.pose_name = pose_name
        self.gripper_action = gripper_action

    def execute(self):
        if self.pose_name is not None:
            self.move_group_obj.go_to(self.pose_name)
        if self.gripper_action == "screw":
            self.move_group_obj.screw()
        if self.gripper_action == "rotation":
            self.move_group_obj.rotate_joint()
        if self.gripper_action == "gear_engagement_1":
            self.move_group_obj.insertion(1, 0.032)
            self.move_group_obj.gear_engagement()
            self.move_group_obj.open_gripper()           
        if self.gripper_action == "gear_engagement_2":
            self.move_group_obj.gear_engagement()
            self.move_group_obj.release_gripper()
        if self.gripper_action == "grasp":
            self.move_group_obj.grasp()
        if self.gripper_action == "grasp_without_gripper":
            self.move_group_obj.grasp_without_gripper()
        if self.gripper_action == "open_gripper":
            self.move_group_obj.open_gripper()
        if self.gripper_action == "change_gripper":
            self.move_group_obj.change_gripper()
        if self.gripper_action == "insertion_1":
            self.move_group_obj.insertion(1.2, 0.021)
            self.move_group_obj.release_gripper()
        if self.gripper_action == "insertion_2":
            self.move_group_obj.insertion(1, 0.0)
            self.move_group_obj.screw()
            self.move_group_obj.open_gripper()
        if self.gripper_action == "insertion_3":
            self.move_group_obj.insertion(1, 0.03)
            self.move_group_obj.release_gripper()
        if self.gripper_action == "move_down":
            self.move_group_obj.move_down()
        if self.gripper_action == "homing_joint_position_1":
            self.move_group_obj.homing_joint_position("1")
        if self.gripper_action == "homing_joint_position_2":
            self.move_group_obj.homing_joint_position("2")
        if self.gripper_action == "release_gripper":
            self.move_group_obj.release_gripper()