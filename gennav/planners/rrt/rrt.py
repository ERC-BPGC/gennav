import math

from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.common import Node
from gennav.utils.geometry import Point


class RRT(Planner):
    """RRT Planner Class.
    Attributes:
        sample_area (tuple): area for sampling random points (min,max)
        sampler (function): function to sample random points in sample_area
        expand_dis (float): distance to expand tree by at each step
        goal_sample_rate (float): rate at which to sample goal during random sampling
    """

    def __init__(self, sample_area, sampler, expand_dis=0.1, goal_sample_rate=0.15):
        """Init RRT Parameters."""

        # RRT.__init__(self)
        self.sample_area = sample_area
        self.sampler = sampler
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate

    def plan(self, start, goal, env):
        """Plans path from start to goal avoiding obstacles.
        Args:
            start_point(gennav.utils.RobotState): with start point coordinates.
            end_point (gennav.utils.RobotState): with end point coordinates.
            env: (gennav.envs.Environment) Base class for an envrionment.
        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
        """

        # Initialize start and goal nodes
        start_node = Node(state=start)
        goal_node = Node(state=goal)

        # Initialize node_list with start
        node_list = [start_node]

        # Loop to keep expanding the tree towards goal if there is no direct connection
        traj = Trajectory(path=[start, goal])
        if not env.get_traj_status(traj):
            while True:
                # Sample random point in specified area
                rnd_point = self.sampler(
                    self.sample_area, goal.position, self.goal_sample_rate
                )

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
                    nearest_node.state.position.y + self.expand_dis * math.sin(theta)
                )
                # Check whether new point is inside an obstacles
                if not env.get_status(RobotState(position=new_point)):
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

                traj = Trajectory(path=[RobotState(position=new_point), goal])
                if distance_to_goal < self.expand_dis or env.get_traj_status(traj):
                    goal_node.parent = node_list[-1]
                    node_list.append(goal_node)
                    print("Goal reached!")
                    break

        else:
            goal_node.parent = start_node
            node_list = [start_node, goal_node]

        # Construct path by traversing backwards through the tree
        path = []
        last_node = node_list[-1]

        while node_list[node_list.index(last_node)].parent is not None:
            node = node_list[node_list.index(last_node)]
            path.append(RobotState(position=node.state.position))
            last_node = node.parent
        path.append(start)
        path = Trajectory(list(reversed(path)))

        return path
