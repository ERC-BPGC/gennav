from gennav.planners.base import Planner
from gennav.utils import Trajectory
from gennav.utils.custom_exceptions import (
    InvalidGoalState,
    InvalidStartState,
    PathNotFound,
)
from gennav.utils.geometry import compute_distance
from gennav.utils.graph import Graph
from gennav.utils.graph_search.astar import astar


class PRM(Planner):
    """PRM Class.

    Args:
        sampler (gennav.utils.sampler.Sampler): sampler to get random states
        r (float): maximum radius to look for neighbours
        n (int): total no. of nodes to be sampled in sample_area
    """

    def __init__(self, sampler, r, n, *args, **kwargs):
        super(PRM, self)
        self.sampler = sampler
        self.r = r
        self.n = n

    def construct(self, env):
        """Constructs PRM graph.

        Args:
            env (gennav.envs.Environment): Base class for an envrionment.

        Returns:
            graph (gennav.utils.graph): A dict where the keys correspond to nodes and
                the values for each key is a list of the neighbour nodes
        """
        nodes = []
        graph = Graph()
        i = 0

        # samples points from the sample space until n points
        # outside obstacles are obtained
        while i < self.n:
            sample = self.sampler()
            if not env.get_status(sample):
                continue
            else:
                i += 1
                nodes.append(sample)

        # finds neighbours for each node in a fixed radius r
        for node1 in nodes:
            for node2 in nodes:
                if node1 != node2:
                    dist = compute_distance(node1.position, node2.position)
                    if dist < self.r:
                        if env.get_traj_status(Trajectory([node1, node2])):
                            if node1 not in graph.nodes:
                                graph.add_node(node1)

                            if node2 not in graph.nodes:
                                graph.add_node(node2)

                            if (
                                node2 not in graph.edges[node1]
                                and node1 not in graph.edges[node2]
                            ):
                                graph.add_edge(
                                    node1, node2,
                                )

        return graph

    def plan(self, start, goal, env):
        """Constructs a graph avoiding obstacles and then plans path from start to goal within the graph.
        Args:
            start (gennav.utils.RobotState): tuple with start point coordinates.
            goal (gennav.utils.RobotState): tuple with end point coordinates.
            env (gennav.envs.Environment): Base class for an envrionment.
        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
            dict: Dictionary containing additional information
        """
        # Check if start and goal states are obstacle free
        if not env.get_status(start):
            raise InvalidStartState(start, message="Start state is in obstacle.")

        if not env.get_status(goal):
            raise InvalidGoalState(goal, message="Goal state is in obstacle.")
        # construct graph
        graph = self.construct(env)
        # find collision free point in graph closest to start_point
        min_dist = float("inf")
        for node in graph.nodes:
            dist = compute_distance(node.position, start.position)
            traj = Trajectory([node, start])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                s = node
        # find collision free point in graph closest to end_point
        min_dist = float("inf")
        for node in graph.nodes:
            dist = compute_distance(node.position, goal.position)
            traj = Trajectory([node, goal])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                e = node
        # add start_point to path
        path = [start]
        traj = Trajectory(path)
        # perform astar search
        p = astar(graph, s, e)
        if len(p.path) == 1:
            raise PathNotFound(p, message="Path contains only one state")
        else:
            traj.path.extend(p.path)
        # add end_point to path
        traj.path.append(goal)
        return traj, {}

    def replan(self, start, goal, env):
        """Constructs a graph avoiding obstacles and then replans path from start to goal within the graph.
        Args:
            start (gennav.utils.RobotState): tuple with start point coordinates.
            goal (gennav.utils.RobotState): tuple with end point coordinates.
            env (gennav.envs.Environment): Base class for an envrionment.
        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
            dict: Dictionary containing additional information
        """
        self.plan(start, goal, env)
