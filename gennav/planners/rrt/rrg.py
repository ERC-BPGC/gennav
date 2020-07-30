import math

from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.geometry import Point
from gennav.utils.graph import Graph
from gennav.utils.graph_search.astar import astar


class RRG(Planner):
    """
    RRT star algorithm
    """

    def __init__(
        self, sample_area, sampler, expand_dis=1.0, goal_sample_rate=0.1, max_iter=1000,
    ):
        """Init RRG parameters

        Args:

            sample_area (tuple): area for sampling random points (min,max)
            sampler (function): samples random points in sample_area
            expand_dis (float, optional): distance to expand tree by at each step. Defaults to 1.0.
            goal_sample_rate (float, optional): rate at which to sample goal during random sampling. Defaults to 0.1.
            max_iter (int, optional): Maximum Iterations. Defaults to 500.
        """
        self.sample_area = sample_area
        self.sampler = sampler
        self.exp_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.graph = Graph()
        self.circle = 1.0

    def plan(self, start, goal, env):
        """Plans path from start to goal avoiding obstacles.
        Args:
            start_point(gennav.utils.RobotState): with start point coordinates.
            end_point (gennav.utils.RobotState): with end point coordinates.
            env: (gennav.envs.Environment) Base class for an envrionment.
        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
        """
        self.graph.add_node(start)

        # Initialize node list with Starting Position
        node_list = [start]

        # Loop for maximum iterations to get the best possible path
        for iter in range(self.max_iter):

            rnd_point = self.sampler(self.sample_area)
            rnd_node = RobotState(position=rnd_point)

            # Find nearest node to the sampled point

            distance_list = [
                (node.position.x - rnd_node.position.x) ** 2
                + (node.position.y - rnd_node.position.y) ** 2
                + (node.position.z - rnd_node.position.z) ** 2
                for node in node_list
            ]

            try:
                # for python2
                nearest_node_index = min(
                    xrange(len(distance_list)), key=distance_list.__getitem__
                )
            except NameError:
                # for python 3
                nearest_node_index = distance_list.index(min(distance_list))
            nearest_node = RobotState()
            nearest_node = node_list[nearest_node_index]

            # Create new point in the direction of sampled point

            theta = math.atan2(
                rnd_node.position.y - nearest_node.position.y,
                rnd_node.position.x - nearest_node.position.x,
            )

            new_node = RobotState(
                position=Point(
                    nearest_node.position.x + self.exp_dis * math.cos(theta),
                    nearest_node.position.y + self.exp_dis * math.sin(theta),
                )
            )
            # Check whether new point is inside an obstacles
            trj = Trajectory([new_node, nearest_node])
            if not env.get_status(new_node):
                if not env.get_traj_status(trj):
                    continue
            else:
                node_list.append(new_node)

            del_x, del_y, del_z = (
                new_node.position.x - goal.position.x,
                new_node.position.y - goal.position.y,
                new_node.position.z - goal.position.z,
            )
            # dist_to_goal = math.sqrt(del_x ** 2 + del_y ** 2 + del_z ** 2)

            # FINDING NEAREST INDICES
            nnode = len(node_list) + 1
            # The circle in which to check parent node and rewiring
            r = self.circle * math.sqrt((math.log(nnode) / nnode))
            dist_list = [
                (rnd_node.position.x - new_node.position.x) ** 2
                + (rnd_node.position.y - new_node.position.y) ** 2
                + (rnd_node.position.z - new_node.position.z) ** 2
                for node in node_list
            ]
            # Getting all the indexes within r units of new_node
            near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
            self.graph.add_node(new_node)
            self.graph.add_edge(nearest_node, new_node)
            # Add all the collision free edges to the graph.
            for ind in near_inds:

                node_check = node_list[ind]
                trj = Trajectory([new_node, node_check])

                # If the above condition is met append the edge.
                if env.get_traj_status(trj):
                    self.graph.add_edge(new_node, node_check)
        if goal not in self.graph.nodes:
            self.graph.add_node(goal)

        path = []
        p = Trajectory([start])
        p = astar(self.graph, start, goal)
        if goal not in p.path:
            path.extend(p.path)
            path.append(goal)
            path = Trajectory(path)
        else:
            path.extend(p.path)
            path = Trajectory(path)

        return path
