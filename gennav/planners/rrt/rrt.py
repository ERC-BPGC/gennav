import math

from matplotlib import pyplot as plt

from descartes import PolygonPatch
from gennav.utils.common import Node
from gennav.utils.planner import check_intersection
from shapely.geometry import Point, Polygon

# from gennav.planners.samplers.samplers import uniform_adjustable_random_sampler as sampler


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
            start_point: Point with start point coordinates.
            end_point: Point with end point coordinates.
            obstacle_list: list of obstacles which themselves are list of points
            animation: flag for showing planning visualization (default False)

        Returns:
            A list of points representing the path determined from
            start to goal while avoiding obstacles.
            A list containing just the start point means path could not be planned.
        """

        # Initialize start and goal nodes
        start = Node.from_coordinates(start_point)
        goal_node = Node.from_coordinates(goal_point)

        # Initialize node_list with start
        node_list = [start]

        # Calculate distances between start and goal
        del_x, del_y, del_z = (
            start.position.x - goal_node.position.x,
            start.position.y - goal_node.position.y,
            start.position.z - goal_node.position.z,
        )
        distance_to_goal = math.sqrt(del_x ** 2 + del_y ** 2 + del_z ** 2)

        # Loop to keep expanding the tree towards goal if there is no direct connection
        if check_intersection([start_point, goal_point], obstacle_list):
            while True:
                # Sample random point in specified area
                rnd_point = self.sampler(
                    self.sample_area, goal_point, self.goal_sample_rate
                )

                # Find nearest node to the sampled point

                distance_list = [
                    (node.position.x - rnd_point.x) ** 2
                    + (node.position.y - rnd_point.y) ** 2
                    + (node.position.z - rnd_point.z) ** 2
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
                    rnd_point.y - nearest_node.position.y, rnd_point.x - nearest_node.x
                )

                new_point = Point()

                new_point.x = (
                    nearest_node.position.x + self.expand_dis * math.cos(theta)
                )
                new_point.y = (
                    nearest_node.position.y + self.expand_dis * math.sin(theta)
                )
                new_point.z = nearest_node.position.z

                # Check whether new point is inside an obstacles
                for obstacle in obstacle_list:
                    if new_point.within(Polygon(obstacle)):
                        new_point.x = float("nan")
                        new_point.y = float("nan")
                        new_point.z = float("nan")
                        continue

                # Expand tree
                if math.isnan(new_point.x):
                    continue
                else:
                    new_node = Node.from_coordinates(new_point)
                    new_node.parent = nearest_node
                    node_list.append(new_node)

                # Check if goal has been reached or if there is direct connection to goal
                del_x, del_y, del_z = (
                    new_node.position.x - goal_node.position.x,
                    new_node.position.y - goal_node.position.y,
                    new_node.position.z - goal_node.position.z,
                )
                distance_to_goal = math.sqrt(del_x ** 2 + del_y ** 2 + del_z ** 2)

                if distance_to_goal < self.expand_dis or not check_intersection(
                    [new_node, goal_node], obstacle_list
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
            path.append(node)
            last_node = node.parent
        path.append(start)

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
            plt.plot(rnd_point.x, rnd_point.y, "^k")

        # Plot each edge of the tree
        for node in node_list:
            if node.parent is not None:
                plt.plot(
                    [node.position.x, node_list[node_list.index(node.parent)].x],
                    [node.position.y, node_list[node_list.index(node.parent)].y],
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
