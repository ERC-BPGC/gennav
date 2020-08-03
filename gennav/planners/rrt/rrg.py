import math

from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.exceptions.custom_exceptions import (
    InvalidGoalState,
    InvalidStartState,
    PathNotFound,
)
from gennav.utils.geometry import Point, compute_distance
from gennav.utils.graph import Graph
from gennav.utils.graph_search.astar import astar


class RRG(Planner):
    """
    RRT star algorithm
    """

    def __init__(
        self, sampler, expand_dis=1.0, max_iter=500,
    ):
        """Init RRG parameters

        Args:

            sampler (gennav.utils.sampler.Sampler): sampler to get random states
            expand_dis (float, optional): distance to expand tree by at each step. Defaults to 1.0.
            max_iter (int, optional): Maximum Iterations. Defaults to 500.
        """
        super(RRG, self).__init__()
        self.sampler = sampler
        self.exp_dis = expand_dis
        self.max_iter = max_iter
        self.circle = 1.0

    def plan(self, start, end, env):
        """Plans path from start to goal avoiding obstacles.
        Args:
            start (gennav.utils.RobotState): Starting state
            end (gennav.utils.RobotState): Goal state
            env: (gennav.envs.Environment) Base class for an envrionment.
        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
        """
        # Check if start and end states are obstacle free
        if not env.get_status(start):
            raise InvalidStartState(start, message="Start state is in obstacle.")

        if not env.get_status(end):
            raise InvalidGoalState(end, message="Goal state is in obstacle.")

        # Initialize graph
        graph = Graph()
        graph.add_node(start)

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
            if not env.get_status(new_node) or not env.get_traj_status(trj):
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
            graph.add_node(new_node)
            graph.add_edge(nearest_node, new_node)
            # Add all the collision free edges to the graph.
            for ind in near_inds:
                node_check = node_list[ind]
                trj = Trajectory([new_node, node_check])
                # If the above condition is met append the edge.
                if env.get_traj_status(trj):
                    graph.add_edge(new_node, node_check)
        if end not in graph.nodes:
            min_cost = 2.0
            closest_node = RobotState()
            for node in graph.nodes:
                temp = compute_distance(node.position, end.position)
                if env.get_traj_status(Trajectory([node, end])) and temp < min_cost:
                    min_cost = temp
                    closest_node = node
            graph.add_edge(closest_node, end)
            graph.add_node(end)
        if start in graph.edges[end]:
            graph.del_edge(start, end)

        path = []
        p = astar(graph, start, end)
        path.extend(p.path)
        if len(path) != 1:
            print("Goal Reached!")
        path = Trajectory(path)

        if len(path.path) == 1:
            raise PathNotFound(path, message="Path contains only one state")

        return path
