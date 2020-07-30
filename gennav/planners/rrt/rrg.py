import math

from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.geometry import Point, compute_distance
from gennav.utils.graph import Graph
from gennav.utils.graph_search.astar import astar


class RRG(Planner):
    """
    RRT star algorithm
    """

    def __init__(
        self, sampler, goal_sample_p=0.1, expand_dis=1.0, max_iter=500,
    ):
        """Init RRG parameters

        Args:

            sampler (function): samples random points in sample_area
            goal_sample_p (float, optional): Probability of sampling goal. Defaults to 0.1
            expand_dis (float, optional): distance to expand tree by at each step. Defaults to 1.0.
            max_iter (int, optional): Maximum Iterations. Defaults to 500.
        """
        self.sampler = sampler
        self.exp_dis = expand_dis
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

            rnd_node = self.sampler()

            # Find nearest node to the sampled point

            distance_list = [
                compute_distance(node.position, rnd_node.position) for node in node_list
            ]

            try:
                # for python2
                nearest_node_index = min(
                    xrange(len(distance_list)), key=distance_list.__getitem__
                )
            except NameError:
                # for python 3
                nearest_node_index = distance_list.index(min(distance_list))
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
            trj = Trajectory([nearest_node, new_node])
            if not env.get_traj_status(trj):
                continue
            else:
                node_list.append(new_node)

            # FINDING NEAREST INDICES
            nnode = len(node_list) + 1
            # The circle in which to check parent node and rewiring
            r = self.circle * math.sqrt((math.log(nnode) / nnode))
            dist_list = [
                compute_distance(node.position, new_node.position) for node in node_list
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
            min_cost = 2.0
            closest_node = RobotState()
            for node in self.graph.nodes:
                temp = compute_distance(node.position, goal.position)
                if env.get_traj_status(Trajectory([node, goal])) and temp < min_cost:
                    min_cost = temp
                    closest_node = node
            self.graph.add_edge(closest_node, goal)
            self.graph.add_node(goal)
        if start in self.graph.edges[goal]:
            self.graph.del_edge(start, goal)

        path = []
        p = astar(self.graph, start, goal)
        path.extend(p.path)
        path = Trajectory(path)

        return path
