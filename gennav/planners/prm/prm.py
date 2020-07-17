import math

from gennav.planners.graph_search.astar import astar
from shapely.geometry import Point, Polygon
from gennav.planners.base import Planner
from gennav.utils.common import Node
from gennav.utils.geometry import Point
from gennav.utils import RobotState, Trajectory

class PRM(Planner):
    """PRM Class.

    Attributes:
        sample_area(tuple): area for sampling random points (min,max)
        sampler(function): function to sample random points in sample_area
        r(float): maximum radius to look for neighbours
        n(int): total no. of nodes to be sampled in sample_area
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
            obstacle_list(list): list of obstacles which themselves are list of points
            animation(bool): flag for showing planning visualization (default False)

        Returns:
            graph(dict):A dict where the keys correspond to nodes and the values for each key is a list
                of the neighbour nodes
        """
        nodes = []
        graph = {}
        i = 0
        # samples points from the sample space until n points
        # outside obstacles are obtained
        while i < self.n:
            sample = self.sampler(self.sample_area)
            if not env.get_status(sample):
                continue
            else:
                i+=1
                nodes.append(sample)

        # finds neighbours for each node in a fixed radius r
        for node1 in nodes:
            for node2 in nodes:
                if node1 != node2:
                    dist = math.sqrt(
                        (node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2
                    )
                    if dist < self.r:
                        traj=Trajectory([RobotState(position=node1),RobotState(position=node2)])
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

    def plan(self, start_point, end_point, env):
        """Constructs a graph avoiding obstacles and then plans path from start to goal within the graph.

        Args:
            start_point(tuple): tuple with start point coordinates.
            end_point(tuple): tuple with end point coordinates.
            obstacle_list(list): list of obstacles which themselves are list of points

        Returns:
            path(list):A list of points representing the path determined from
                    start to goal.An list containing just the start point means
                     path could not be planned.
        """
        # construct graph
        graph = self.construct(env)
        # find collision free point in graph closest to start_point
        min_dist = float("inf")
        for node in graph.keys():
            dist = math.sqrt(
                (node.x - start_point.x) ** 2 + (node.y - start_point.y) ** 2
            )
            traj=Trajectory([RobotState(position=node),RobotState(position=start_point)])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                s = node
        # find collision free point in graph closest to end_point
        min_dist = float("inf")
        for node in graph.keys():
            dist = math.sqrt(
                (node.x - end_point.x) ** 2 + (node.y - end_point.y) ** 2
            )
            traj=Trajectory([RobotState(position=node),RobotState(position=end_point)])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                e = node
        # add start_point to path
        path = [RobotState(position=start_point)]
        # perform astar search
        p=astar(graph, s, e)
        if len(p)==1:
            return Trajectory(path)
        else:
            path.extend(p)
        # add end_point to path
        path.append(RobotState(position=end_point))
        return Trajectory(path)
