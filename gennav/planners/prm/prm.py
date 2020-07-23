import math

from gennav.planners.base import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.graph_search.astar import astar


class PRM(Planner):
    """PRM Class.

    Attributes:
        sample_area (tuple): area for sampling random points (min,max)
        sampler (function): function to sample random points in sample_area
        r (float): maximum radius to look for neighbours
        n (int): total no. of nodes to be sampled in sample_area
    """

    def __init__(self, sample_area, sampler, r, n):
        """Init PRM Parameters."""

        self.sample_area = sample_area
        self.sampler = sampler
        self.r = r
        self.n = n

    def construct(self, env):
        """Constructs PRM graph.

        Args:
            env (gennav.envs.Environment): Base class for an envrionment.

        Returns:
            graph (dict): A dict where the keys correspond to nodes and
                the values for each key is a list of the neighbour nodes
        """
        nodes = []
        graph = {}
        i = 0
        # samples points from the sample space until n points
        # outside obstacles are obtained
        while i < self.n:
            sample = self.sampler(self.sample_area)
            if not env.get_status(RobotState(position=sample)):
                continue
            else:
                i += 1
                nodes.append(sample)

        # finds neighbours for each node in a fixed radius r
        for node1 in nodes:
            for node2 in nodes:
                if node1 != node2:
                    dist = math.sqrt(
                        (node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2
                    )
                    if dist < self.r:
                        traj = Trajectory(
                            [RobotState(position=node1), RobotState(position=node2)]
                        )
                        if env.get_traj_status(traj):
                            if node1 not in graph:
                                graph[node1] = [node2]
                            elif node2 not in graph[node1]:
                                graph[node1].append(node2)
                            if node2 not in graph:
                                graph[node2] = [node1]
                            elif node1 not in graph[node2]:
                                graph[node2].append(node1)

        return graph

    def plan(self, start, goal, env):
        """Constructs a graph avoiding obstacles and then plans path from start to goal within the graph.

        Args:
            start (gennav.utils.RobotState): tuple with start point coordinates.
            goal (gennav.utils.RobotState): tuple with end point coordinates.
            env (gennav.envs.Environment): Base class for an envrionment.

        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
        """
        # construct graph
        graph = self.construct(env)
        # find collision free point in graph closest to start_point
        min_dist = float("inf")
        for node in graph.keys():
            dist = math.sqrt(
                (node.x - start.position.x) ** 2 + (node.y - start.position.y) ** 2
            )
            traj = Trajectory(
                [RobotState(position=node), RobotState(position=start.position)]
            )
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                s = node
        # find collision free point in graph closest to end_point
        min_dist = float("inf")
        for node in graph.keys():
            dist = math.sqrt(
                (node.x - goal.position.x) ** 2 + (node.y - goal.position.y) ** 2
            )
            traj = Trajectory(
                [RobotState(position=node), RobotState(position=goal.position)]
            )
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                e = node
        # add start_point to path
        path = [start]
        traj = Trajectory(path)
        # perform astar search
        p = astar(graph, s, e)
        if len(p.path) == 1:
            return traj
        else:
            traj.path.extend(p.path)
        # add end_point to path
        traj.path.append(goal)
        return traj
