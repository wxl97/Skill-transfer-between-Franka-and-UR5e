"""

Path planning Sample Code with Randomized Rapidly-Exploring Random
Trees with sobol low discrepancy sampler(RRTSobol).
Sobol wiki https://en.wikipedia.org/wiki/Sobol_sequence

The goal of low discrepancy samplers is to generate a sequence of points that
optimizes a criterion called dispersion.  Intuitively, the idea is to place
samples to cover the exploration space in a way that makes the largest
uncovered area be as small as possible.  This generalizes of the idea of grid
resolution.  For a grid, the resolution may be selected by defining the step
size for each axis.  As the step size is decreased, the resolution increases.
If a grid-based motion planning algorithm can increase the resolution
arbitrarily, it becomes resolution complete.  Dispersion can be considered as a
powerful generalization of the notion of resolution.

Taken from
LaValle, Steven M. Planning algorithms. Cambridge university press, 2006.


authors:
    First implementation AtsushiSakai(@Atsushi_twi)
    Sobol sampler Rafael A.
Rojas (rafaelrojasmiliani@gmail.com)


"""

import math
import random
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
import matplotlib.pyplot as plt
import numpy as np

from RRTplanner.sobol import sobol_quasirand



class RRTSobol:
    """
    Class for RRTSobol planning
    """

    class Node:
        """
        RRTSobol Node
        """

        def __init__(self, *coords):
            self.coords = list(coords)  # Support N-D coordinates
            self.path_coords = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=0.05,
                 path_resolution=0.05,  # Smaller resolution for denser path
                 goal_sample_rate=20,  # Increased goal sampling rate
                 max_iter=5000,
                 robot_radius=0.0):
        """
        Setting Parameter

        start:Start Position [x,y,...]
        goal:Goal Position [x,y,...]
        obstacle_list:obstacle Positions [[x,y,...,size],...]
        randArea:Random Sampling Area [min,max]
        robot_radius: robot body modeled as circle with given radius

        """
        self.dof = len(start)  # Degrees of freedom
        self.start = self.Node(*start)
        self.end = self.Node(*goal)
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.sobol_inter_ = 0
        self.robot_radius = robot_radius

    def planning(self):
        """
        rrt path planning
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)


            if self.calc_dist_to_goal(self.node_list[-1].coords) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(
                        final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(*from_node.coords)
        d, direction = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_coords = [list(new_node.coords)]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.coords = [c + self.path_resolution * dir for c, dir in zip(new_node.coords, direction)]
            new_node.path_coords.append(list(new_node.coords))

        # Ensure the final point is added
        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_coords.append(list(to_node.coords))
            new_node.coords = list(to_node.coords)

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [self.end.coords]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node.coords)
            node = node.parent
        path.append(node.coords)
        path = path[::-1]  # Reverse the path to start from the beginning

        return path

    def calc_dist_to_goal(self, coords):
        diff = [c - e for c, e in zip(coords, self.end.coords)]
        return math.sqrt(sum(x**2 for x in diff))

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rand_coordinates, n = sobol_quasirand(self.dof, self.sobol_inter_)
            rand_coordinates = self.min_rand + \
                rand_coordinates * (self.max_rand - self.min_rand)
            self.sobol_inter_ = n
            rnd = self.Node(*rand_coordinates)
        else:  # goal point sampling
            rnd = self.Node(*self.end.coords)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [sum((c1 - c2)**2 for c1, c2 in zip(node.coords, rnd_node.coords))
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacle_list, robot_radius):
        if node is None:
            return False

        for obstacle in obstacle_list:
            center, size = obstacle[:-1], obstacle[-1]
            d_list = [sum((c - n)**2 for c, n in zip(center, coords)) for coords in node.path_coords]
            if min(d_list) <= (size + robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        diff = [to - fr for fr, to in zip(from_node.coords, to_node.coords)]
        d = math.sqrt(sum(x**2 for x in diff))
        theta = [x / d for x in diff] if d != 0 else [0] * len(diff)
        return d, theta


def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        diff = [p2 - p1 for p1, p2 in zip(path[i], path[i + 1])]
        d = math.sqrt(sum(x**2 for x in diff))
        le += d
    return le


def get_target_point(path, targetL):
    le = 0
    ti = 0
    last_pair_len = 0
    for i in range(len(path) - 1):
        diff = [p2 - p1 for p1, p2 in zip(path[i], path[i + 1])]
        d = math.sqrt(sum(x**2 for x in diff))
        if d < 1e-6:  # 如果两点几乎重合，则跳过
            continue
        le += d
        if le >= targetL:
            ti = i
            last_pair_len = d
            break

    if last_pair_len < 1e-6:  # 如果所有点都重合，直接返回最后一个点
        return path[-1], len(path) - 1

    part_ratio = (le - targetL) / last_pair_len
    target = [p1 + (p2 - p1) * part_ratio for p1, p2 in zip(path[ti], path[ti + 1])]
    return target, ti


def line_collision_check(first, second, obstacle_list, robot_radius):
    for obstacle in obstacle_list:
        center, size = obstacle[:-1], obstacle[-1]
        for t in np.linspace(0, 1, num=100):  # Check points along the line
            point = [f + t * (s - f) for f, s in zip(first, second)]
            dist = math.sqrt(sum((c - p)**2 for c, p in zip(center, point)))
            if dist <= size + robot_radius:
                return False
    return True


def path_smoothing(path, max_iter, obstacle_list, robot_radius):
    if len(path) < 2:
        return path  # 路径太短，无法平滑

    le = get_path_length(path)
    if le < 1e-6:
        return path  # 路径总长度接近0，无需平滑

    for _ in range(max_iter):
        pick_points = [random.uniform(0, le), random.uniform(0, le)]
        pick_points.sort()
        first, first_index = get_target_point(path, pick_points[0])
        second, second_index = get_target_point(path, pick_points[1])

        if first_index <= 0 or second_index <= 0 or second_index == first_index:
            continue

        if not line_collision_check(first, second, obstacle_list, robot_radius):
            continue

        # === 关键：用直线段替换中间段 ===
        # 保留前first_index的点，加first，加second，保留second_index+1及之后的点
        new_path = []
        new_path.extend(path[:first_index])
        new_path.append(first)
        new_path.append(second)
        new_path.extend(path[second_index+1:])
        path = new_path
        le = get_path_length(path)  # 更新总长度

    return path


def densify_path(path, max_segment_length=0.3):
    new_path = [path[0]]
    for i in range(1, len(path)):
        prev = new_path[-1]
        curr = path[i]
        dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(prev, curr)))
        n_split = int(math.ceil(dist / max_segment_length))
        for j in range(1, n_split + 1):
            interp = [a + (b - a) * j / n_split for a, b in zip(prev, curr)]
            new_path.append(interp)
    return new_path


def is_in_box(point, box_center, box_size):
    """
    判断点point(x,y,z)是否在以box_center为中心，box_size为长宽高的立方体内
    """
    for i in range(3):
        if abs(point[i] - box_center[i]) > box_size[i] / 2.0:
            return False
    return True

def main():
    print("start " + __file__)

    # ==== 选择关节数 ====
    dof = 7  # 可改为6或其他，根据实际机器人选择

    # ====Search Path with RRTSobol====
    if dof == 7:
        target_pose = [-0.5945910223071946, 0.4301502852490936, 0.136105098321691, -1.6266413795849624, -0.01258050409088198, 2.06070846985867, 0.3387359455803625]
        start_pose = [-0.382178, -0.643689, 0.278861, -2.401447, 0.176929, 1.800268, 0.599072]
        obstacle_list = [
            [0.45, -0.15, 0.45, -0.92, 0.39, -0.01, 0.01, 0.02],
            [0.55, -0.25, 0.40, -0.92, 0.39, -0.01, 0.02, 0.02],
        ]
    elif dof == 6:
        target_pose = [-0.59, 0.43, 0.13, -1.62, -0.01, 2.06]
        start_pose = [-0.38, -0.64, 0.27, -2.40, 0.17, 1.80]
        obstacle_list = [
            [0.45, -0.15, 0.45, -0.92, 0.39, 0.01, 0.02],
            [0.55, -0.25, 0.40, -0.92, 0.39, 0.02, 0.02],
        ]
    else:
        raise ValueError("Unsupported DOF")

    workspace_obstacle = {
        "center": [0.35, 0.0, 0.3],  # x, y, z
        "size": [0.1, 0.15, 0.15]    # 长宽高
    }

    # Set Initial parameters
    rrt = RRTSobol(
        start=start_pose,
        goal=target_pose,
        rand_area=[-3.14, 3.14],  # Adjusted sampling area
        obstacle_list=obstacle_list,
        robot_radius=0.05,  # Robot radius
        goal_sample_rate=20,  # Increased goal sampling rate
        path_resolution=0.05  # Smaller resolution for denser path
    )
    path = rrt.planning()
    
    if path is None:
        print("Cannot find path")
    else:
        # Remove duplicate points in the path
        path = [p for i, p in enumerate(path) if i == 0 or p != path[i - 1]]
        print(f"Original path length: {get_path_length(path):.4f}, points: {len(path)}")
        
        # Apply path smoothing
        max_iter = 1000
        smoothed_path = path_smoothing(path, max_iter, obstacle_list, rrt.robot_radius)
        print(f"Smoothed path length: {get_path_length(smoothed_path):.4f}, points: {len(smoothed_path)}")
        print(smoothed_path)



if __name__ == '__main__':
    main()
