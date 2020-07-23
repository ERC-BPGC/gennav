import math

from gennav.planners.base import Planner
from gennav.planners.graph_search.astar import astar
from gennav.utils import RobotState, Trajectory
from gennav.utils.geometry import compute_distance


class PRMStar(Planner):
    """PRM-Star Class.

    Args:
        sampler (gennav.utils.sampler.Sampler): sampler to get random states
        c (float): a constant for radius determination
        n (int): total no. of nodes to be sampled in sample_area
    """

    def __init__(self, sampler, c, n):
        self.sampler = sampler
        self.n = n
        self.c = c

    def construct(self, env):
        """Constructs PRM-Star graph.

        Args:
            env (gennav.envs.Environment): Base class for an envrionment.

        Returns:
            graph (dict): A dict where the keys correspond to nodes and
                the values for each key is a list of the neighbour nodes
        """
        points = []
        graph = {}
        i = 0

        # samples points from the sample space until n points
        # outside obstacles are obtained
        while i < self.n:
            sample = self.sampler()
            if not env.get_status(sample):
                continue
            else:
                i += 1
                points.append(sample.position)

        # finds neighbours for each node in a fixed radius r
        r = self.c * math.sqrt(math.log(self.n) / self.n)
        for p1 in points:
            for p2 in points:
                if p1 != p2:
                    dist = compute_distance(p1, p2)
                    if dist < r:
                        traj = Trajectory(
                            path=[RobotState(position=p1), RobotState(position=p2)]
                        )
                        if env.get_traj_status(traj):
                            if p1 not in graph:
                                graph[p1] = [p2]
                            elif p2 not in graph[p1]:
                                graph[p1].append(p2)
                            if p2 not in graph:
                                graph[p2] = [p1]
                            elif p1 not in graph[p2]:
                                graph[p2].append(p1)

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
