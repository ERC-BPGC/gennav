import math

from gennav.envs import Environment
from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.common import Node
from gennav.utils.geometry import Point
# from gennav.utils.samplers import uniform_adjustable_random_sampler as sampler

env = Environment()


class RRT(Planner):
    """RRT Planner Class.

    Attributes:
        sample_area: area for sampling random points (min,max)
        sampler: function to sample random points in sample_area
        expand_dis: distance to expand tree by at each step
        goal_sample_rate: rate at which to sample goal during random sampling
    """

    def __init__(self, sample_area, sampler, expand_dis=0.1, goal_sample_rate=0.15):
        """Init RRT Parameters."""

        RRT.__init__(self)
        self.sample_area = sample_area
        self.sampler = sampler
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate

    def plan(self, start_point, goal_point, env):
        """Plans path from start to goal avoiding obstacles.

        Args:
            start_point: Point (from gennav.utils.geometry) with start point coordinates.
            end_point: Point (from gennav.utils.geometry) with end point coordinates.
            env: (gennav.envs.Environment) Base class for an envrionment.
        Returns:
            A list of points (Point from gennav.utils.geometry) representing the path determined from
            start to goal while avoiding obstacles.
            A list containing just the start point (Point from gennav.utils.geometry) means path could not be planned.
        """

        # Initialize start and goal nodes
        start = Node.from_coordinates(start_point)
        goal_node = Node.from_coordinates(goal_point)

        # Initialize node_list with start
        node_list = [start]

        # Calculate distances between start and goal
        del_x, del_y, del_z = (
            start.state.position.x - goal_node.state.position.x,
            start.state.position.y - goal_node.state.position.y,
            start.state.position.z - goal_node.state.position.z,
        )
        distance_to_goal = math.sqrt(del_x ** 2 + del_y ** 2 + del_z ** 2)

        # Loop to keep expanding the tree towards goal if there is no direct connection
        traj = Trajectory(
            [RobotState(position=start_point), RobotState(position=goal_point)]
        )
        if not env.get_traj_status(traj):
            while True:
                # Sample random point in specified area
                rnd_point = self.sampler(self.sample_area, goal_point, self.goal_sample_rate)

                # Find nearest node to the sampled point

                distance_list = [
                    (node.state.position.x - rnd_point.x) ** 2
                    + (node.state.position.y - rnd_point.y) ** 2
                    + (node.state.position.z - rnd_point.z) ** 2
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
                    rnd_point.y - nearest_node.state.position.y,
                    rnd_point.x - nearest_node.state.position.x,
                )

                new_point = Point()

                new_point.x = (
                    nearest_node.state.position.x + self.expand_dis * math.cos(theta)
                )
                new_point.y = (
                    nearest_node.state.position.y + self.expand_dis * math.sin(theta),
                )

                # Check whether new point is inside an obstacles
                if not env.get_status(RobotState(position=new_point)):
                    new_point.x = float("nan")
                    new_point.y = float("nan")
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
                    new_node.state.position.x - goal_node.state.position.x,
                    new_node.state.position.y - goal_node.state.position.y,
                    new_node.state.position.z - goal_node.state.position.z,
                )
                distance_to_goal = math.sqrt(del_x ** 2 + del_y ** 2 + del_z ** 2)

                traj = Trajectory(
                    [RobotState(position=new_point), RobotState(position=goal_point)]
                )
                if distance_to_goal < self.expand_dis or env.get_traj_status(traj):
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

        return list(reversed(path)), node_list
