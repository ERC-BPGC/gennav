import math

from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.common import Node
from gennav.utils.custom_exceptions import (
    InvalidGoalState,
    InvalidStartState,
    PathNotFound,
)
from gennav.utils.geometry import Point, compute_distance


class RRT(Planner):
    """RRT Planner Class.

    Args:
        sampler (gennav.utils.sampler.Sampler): sampler to get random states
        expand_dis (float): distance to expand tree by at each step
    """

    def __init__(self, sampler, expand_dis=0.1):
        super(RRT, self).__init__()
        self.sampler = sampler
        self.expand_dis = expand_dis

    def plan(self, start, goal, env):
        """Plans path from start to goal avoiding obstacles.

        Args:
            start_point(gennav.utils.RobotState): with start point coordinates.
            end_point (gennav.utils.RobotState): with end point coordinates.
            env: (gennav.envs.Environment) Base class for an envrionment.

        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
        """
        # Check if start and goal states are obstacle free
        if not env.get_status(start):
            raise InvalidStartState(start, message="Start state is in obstacle.")

        if not env.get_status(goal):
            raise InvalidGoalState(goal, message="Goal state is in obstacle.")

        # Initialize start and goal nodes
        start_node = Node(state=start)
        goal_node = Node(state=goal)

        # Initialize node_list with start
        node_list = [start_node]

        # Loop to keep expanding the tree towards goal if there is no direct connection
        traj = Trajectory(path=[start, goal])
        if not env.get_traj_status(traj):
            while True:
                # Sample random state in specified area
                rnd_state = self.sampler()

                # Find nearest node to the sampled point
                distance_list = [
                    compute_distance(node.state.position, rnd_state.position)
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
                    rnd_state.position.y - nearest_node.state.position.y,
                    rnd_state.position.x - nearest_node.state.position.x,
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
                distance_to_goal = compute_distance(
                    goal_node.state.position, new_node.state.position
                )

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
        while last_node.parent is not None:
            path.append(last_node.state)
            last_node = last_node.parent
        path.append(start)
        path = Trajectory(path[::-1])

        if len(path.path) == 1:
            raise PathNotFound(path, message="Path contains only one state")

        return path
