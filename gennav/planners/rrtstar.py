import math
import random

from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.common import Node
from gennav.utils.geometry import Point, compute_distance, compute_angle


# Pseudo Code: Sampling based motion planning algorithm
# 1. Initialize starting nodes, obstacles, goal(end point to be reached) & random sampling area
# 2. Generate random nodes
# 3. Calculate nearest nodes from randomly generated nodes
# 4. Steer/Generate new path from these nodes
# 5. Check if newly generated nodes from steer path are collision free
# 6. Check for their optimal cost
# 7. Create the minimum cost path from these nodes using above steps recursively
# 8. Rewire tree to connect nearest nodes

show_animation = True


class RRTStar:
    """Class for RRT Star planning.

    """

    class Node:
        """Class for representing node form

        """

        def __init__(self, x, y):
            """ Initialize attributes of node

            Attributes:
                x,y (float): X & Y coordinates
                path_x,y (list): X & Y coordinates of path to this node 
                parent (Node) : Node connected to present node
                cost (float): cost/distance suffered to get to this node

            """

            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0

    def __init__(
        self,
        start,
        goal,
        obstacle_list,
        rand_area,
        expand_dis=30.0,
        path_resolution=1.0,
        goal_sample_rate=20,
        max_iter=300,
        search_until_max_iter=True,
        connect_circle_dist=50.0,
    ):
        """
        Setting initial Parameters

        Attributes:
            start: Start Position [x,y], default=(0,0)
            goal: Goal Position coordinates [x,y] to be reached 
            obstacleList: obstacle Positions [[x,y,obj_size],...]
            randArea: Random Sampling Area [min,max]
            goal_sample_rate: Rate at which goal should be sampled to drive towards goal
            max_iter: Maximum Iterations to be performed
            search_until_max_iter: If False then gets the path as soon as found
            connect_circle_dist: Variable distance to get smoother and shorter path.
                                  On increasing Path Smoothens, but Processing Increases as well
        """
        self.start = self.Node(
            start[0], start[1]
        )  # Initialzing starting position as initial nodes
        self.end = self.Node(
            goal[0], goal[1]
        )  # Initialzing ending position as initial goal
        self.obstacle_list = obstacle_list
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_iter = max_iter
        self.search_until_max_iter = search_until_max_iter
        self.connect_circle_dist = connect_circle_dist
        self.node_list = []
        self.goal_node = self.Node(goal[0], goal[1])

        @staticmethod
    def check_collision(node, obstacleList):
        """Check if nodes are colliding with obstacles
        Args:
            node : Node class 
            obstacleList (list) : List of obstacles storing their coordinates and obj_size

        Returns:
            bool: True if collision is found, False otherwise.

        """

        # If no nodes are present then obviously no collision is possible
        if node is None:
            return False

        for (obj_x, obj_y, obj_size) in obstacleList:
            dx_list = [obj_x - x for x in node.path_x]
            dy_list = [obj_y - y for y in node.path_y]
            dist_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            # If least square of distance is less than square of object size then they are colliding
            if min(dist_list) <= obj_size ** 2:
                return False  # collision

        return True  # safe

    def get_random_node(self):
        """Generates any random node

        Returns: 
            node: random_node. Randomly generated node

        """
        if random.randint(0, 100) > self.goal_sample_rate:
            random_node = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
            )
        else:  # goal point sampling
            random_node = self.Node(self.end.x, self.end.y)
        return random_node

    def generate_final_course(self, goal_index):
        """Generates final path/course from nodes to goal 

        Args:
             goal_index (int): Index of goal's node 

        Returns:
            list: path. Variable containing all coordinates of node to make path in order. 
        """
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_index]
        while node.parent is not None:
            path.append([node.x, node.y])  # Adding nodes to the path
            node = node.parent
        path.append([node.x, node.y])

        return path

    @staticmethod
    def get_nearest_node_index(node_list, random_node):
        """Finds the nearest node from a random node
        
        Args:
            node_list (list): List of all the nodes
            random_node : Any randomly generated node 
        Returns:
            int: minind. Minimum index/ index of minimum distance node 
        """

        dist_list = [
            (node.x - random_node.x) ** 2 + (node.y - random_node.y) ** 2
            for node in node_list
        ]
        minind = dist_list.index(min(dist_list))

        return minind

    def steer(self, initial_node, to_node, extend_length=float("inf")):
        """Steers a new path based on optimizing cost & angle

        Args:
            extend_length (float): We initially take lowest path cost as infinite

        Returns:
            node: new_node. The new calculated node based on path optimization

        """
        new_node = Node(initial_node.x, initial_node.y)
        cost = compute_distance(new_node, to_node)
        theta = compute_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > cost:
            extend_length = cost

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        cost, _ = self.calc_distance_and_angle(new_node, to_node)
        if cost <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = initial_node

        return new_node

    def planning(self, animation=True, search_until_max_iter=True):
        """
        Path planing 

        Args:
            animation: flag for animation on or off
            search_until_max_iter: search until max iteration for path improving or not

        Returns:
               Final Path/Course planned
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            random_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, random_node)
            new_node = self.steer(
                self.node_list[nearest_ind], random_node, self.expand_dis
            )

            if self.check_collision(new_node, self.obstacle_list):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)

            # Snippet to draw/animate node
            # if animation and i % 5 == 0:
            #     self.draw_graph(random_node)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        # Snippet to be executed if max iterations is reached before final
        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_indexes):
        """Choosing new parent nodes based on collision & cost checks

        Returns:
            node: new_node. New parent node if found, None otherwise
        """

        if not near_indexes:
            return None

        # search nearest cost in near_indexes
        costs = []
        for i in near_indexes:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_indexes[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        """Searching best/optimal goal node based on colliison & cost checks

        Returns:
            int: i. Index value of goal node if safe/optimal goal found, None otherwise

        """

        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_indexes = [
            dist_to_goal_list.index(i)
            for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_indexes = []
        for goal_index in goal_indexes:
            t_node = self.steer(self.node_list[goal_index], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list):
                safe_goal_indexes.append(goal_index)

        if not safe_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_indexes])
        for i in safe_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """Finding nearest located nodes

        Returns:
            list: near_indexes. Int list of indexes of nearest nodes

        """
        number_nodes = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(
            (math.log(number_nodes) / number_nodes)
        )

        # if expand_dist exists, search vertices in a range no more than expand_dist
        if hasattr(self, "expand_dis"):
            r = min(r, self.expand_dis)
        dist_list = [
            (node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2
            for node in self.node_list
        ]
        near_indexes = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_indexes

    def rewire(self, new_node, near_indexes):
        """"Rewiring path by checking if cost is optimal & no collision is taking place

        """

        for i in near_indexes:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, initial_node, to_node):
        """New cost calculated/obtained after path change

        """

        cost = compute_distance(initial_node, to_node)
        return initial_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        """Cost incurred to tree leave from each node calculated/updated recursively

        """

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)
