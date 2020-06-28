import math

import matplotlib.pyplot as plt
from descartes import PolygonPatch
from shapely.geometry import Point, Polygon

from gennav.utils.planner import check_intersection

# from gennav.planners.samplers.samplers import uniform_adjustable_random_sampler as sampler


class Node:
    """Node for RRT.

    Attributes:
        x (float): x-coordinate of the node.
        y (float): y-coordinate of the node.
        parent (Node): parent node of current node.
    """

    def __init__(self, x, y):
        """
        Init node with x and y coordinates.
        """
        self.x = x
        self.y = y
        self.parent = None

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"

    @classmethod
    def from_coordinates(cls, coordinate):
        """Create Node from tuple with coordinates."""
        return cls(x=coordinate[0], y=coordinate[1])

    def to_tuple(self):
        return self.x, self.y


class RRT:
    """RRT Planner Class.

    Attributes:
        sample_area: area for sampling random points (min,max)
        sampler: function to sample random points in sample_area
        expand_dis: distance to expand tree by at each step
        goal_sample_rate: rate at which to sample goal during random sampling
    """

    def __init__(self, sample_area, sampler, expand_dis=0.1, goal_sample_rate=0.15):
        """Init RRT Parameters."""

        self.sample_area = sample_area
        self.sampler = sampler
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate

    def plan(self, start_point, goal_point, obstacle_list, animation=False):
        """Plans path from start to goal avoiding obstacles.

        Args:
            start_point: tuple with start point coordinates.
            end_point: tuple with end point coordinates.
            obstacle_list: list of obstacles which themselves are list of points
            animation: flag for showing planning visualization (default False)

        Returns:
            A list of points representing the path determined from
            start to goal while avoiding obstacles.
            An list containing just the start point means path could not be planned.
        """

        # Initialize start and goal nodes
        start = Node.from_coordinates(start_point)
        goal_node = Node.from_coordinates(goal_point)

        # Initialize node_list with start
        node_list = [start]

        # Calculate distances between start and goal
        del_x, del_y = start.x - goal_node.x, start.y - goal_node.y
        distance_to_goal = math.sqrt(del_x ** 2 + del_y ** 2)

        # Loop to keep expanding the tree towards goal if there is no direct connection
        if check_intersection([start_point, goal_point], obstacle_list):
            while True:
                # Sample random point in specified area
                rnd_point = self.sampler(
                    self.sample_area, goal_point, self.goal_sample_rate
                )

                # Find nearest node to the sampled point
                distance_list = [
                    (node.x - rnd_point[0]) ** 2 + (node.y - rnd_point[1]) ** 2
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

                nearest_node = node_list[nearest_node_index]

                # Create new point in the direction of sampled point
                theta = math.atan2(
                    rnd_point[1] - nearest_node.y, rnd_point[0] - nearest_node.x
                )
                new_point = (
                    nearest_node.x + self.expand_dis * math.cos(theta),
                    nearest_node.y + self.expand_dis * math.sin(theta),
                )

                # Check whether new point is inside an obstacles
                for obstacle in obstacle_list:
                    if Point(new_point).within(Polygon(obstacle)):
                        new_point = float("nan"), float("nan")
                        continue

                # Expand tree
                if math.isnan(new_point[0]):
                    continue
                else:
                    new_node = Node.from_coordinates(new_point)
                    new_node.parent = nearest_node
                    node_list.append(new_node)

                # Check if goal has been reached or if there is direct connection to goal
                del_x, del_y = new_node.x - goal_node.x, new_node.y - goal_node.y
                distance_to_goal = math.sqrt(del_x ** 2 + del_y ** 2)

                if distance_to_goal < self.expand_dis or not check_intersection(
                    [new_node.to_tuple(), goal_node.to_tuple()], obstacle_list
                ):
                    goal_node.parent = node_list[-1]
                    node_list.append(goal_node)
                    print("Goal reached!")
                    break

        else:
            goal_node.parent = start
            node_list = [start, goal_node]

        # Construct path by traversing backwards through the tree
        path = []
        last_node = node_list[-1]

        while node_list[node_list.index(last_node)].parent is not None:
            node = node_list[node_list.index(last_node)]
            path.append(node.to_tuple())
            last_node = node.parent
        path.append(start.to_tuple())

        if animation is True:
            RRT.visualize_tree(node_list, obstacle_list)

        return list(reversed(path)), node_list

    @staticmethod
    def visualize_tree(node_list, obstacle_list, rnd_point=None):
        """Draw the tree along with randomly sampled point.

            Args:
                node_list: list of nodes in the tree.
                obstacle_list: list of obstactles.
                rnd_point: randomly sampled point.

            Returns:
                Nothing. Function is used to draw the tree.
        """

        # Clear the figure
        plt.clf()

        # Plot randomly sampled point
        if rnd_point is not None:
            plt.plot(rnd_point[0], rnd_point[1], "^k")

        # Plot each edge of the tree
        for node in node_list:
            if node.parent is not None:
                plt.plot(
                    [node.x, node_list[node_list.index(node.parent)].x],
                    [node.y, node_list[node_list.index(node.parent)].y],
                    "-g",
                )

        # Draw the obstacles in the environment
        for obstacle in obstacle_list:
            obstacle_polygon = Polygon(obstacle)
            fig = plt.figure(1, figsize=(5, 5), dpi=90)
            ax = fig.add_subplot(111)
            poly_patch = PolygonPatch(obstacle_polygon)
            ax.add_patch(poly_patch)

        plt.show()
