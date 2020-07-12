import math

from descartes import PolygonPatch
from gennav.planners.graph_search_algorithms.astar import astar
from gennav.utils.planner import check_intersection
from matplotlib import pyplot as plt
from shapely.geometry import Point, Polygon


class PRM:
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

    def construct(self, obstacle_list, animation=False):
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
            for obstacle in obstacle_list:
                if Point(sample).within(Polygon(obstacle)):
                    sample = float("nan"), float("nan")
                    break
            if math.isnan(sample[0]):
                continue
            else:
                i += 1
                nodes.append(sample)
        # finds neighbours for each node in a fixed radius r
        for node1 in nodes:
            for node2 in nodes:
                if node1 != node2:
                    dist = math.sqrt(
                        (node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2
                    )
                    if dist < self.r:
                        if not check_intersection([node1, node2], obstacle_list):
                            if node1 not in graph:
                                graph[node1] = [node2]
                            elif node2 not in graph[node1]:
                                graph[node1].append(node2)
                            if node2 not in graph:
                                graph[node2] = [node1]
                            elif node1 not in graph[node2]:
                                graph[node2].append(node1)
        if animation is True:
            PRM.visualize_graph(graph, obstacle_list)

        return graph

    def plan(self, start_point, end_point, obstacle_list):
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
        graph = self.construct(obstacle_list)
        # find collision free point in graph closest to start_point
        min_dist = float("inf")
        for node in graph.keys():
            dist = math.sqrt(
                (node[0] - start_point[0]) ** 2 + (node[1] - start_point[1]) ** 2
            )
            if dist < min_dist and (
                not check_intersection([node, start_point], obstacle_list)
            ):
                min_dist = dist
                s = node
        # find collision free point in graph closest to end_point
        min_dist = float("inf")
        for node in graph.keys():
            dist = math.sqrt(
                (node[0] - end_point[0]) ** 2 + (node[1] - end_point[1]) ** 2
            )
            if dist < min_dist and (
                not check_intersection([node, end_point], obstacle_list)
            ):
                min_dist = dist
                e = node
        # add start_point to path
        path = [start_point]
        # perform astar search
        path.extend(astar(graph, s, e))
        # add end_point to path
        path.append(end_point)
        return path

    @staticmethod
    def visualize_graph(graph, obstacle_list):
        """Draw the graph.

            Args:
                graph(dict): the dict representing the graph
                obstacle_list(list): list of obstactles.

            Returns:
                Nothing. Function is used to draw the graph.
        """
        # Clears the figure
        plt.clf()
        # Plot each edge of the tree
        for node in graph:
            for neighbour in graph[node]:
                plt.plot(
                    [node[0], neighbour[0]], [node[1], neighbour[1]], "-g",
                )

        # Draw the obstacles in the environment
        for obstacle in obstacle_list:
            obstacle_polygon = Polygon(obstacle)
            fig = plt.figure(1, figsize=(5, 5), dpi=90)
            ax = fig.add_subplot(111)
            poly_patch = PolygonPatch(obstacle_polygon)
            ax.add_patch(poly_patch)

        plt.show()
