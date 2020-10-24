import numpy as np

from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.common import Node
from gennav.utils.custom_exceptions import (
    InvalidGoalState,
    InvalidStartState,
    PathNotFound,
)
from gennav.utils.geometry import Point, compute_distance


class RRTConnect(Planner):
    """RRTConnect Planner Class.

    Drawback: A lot of nearest neighbour searches are performed. 
              Should use kd-trees to optimize this

    Args:
        sampler (gennav.utils.sampler.Sampler): sampler to get random states
        expand_dis (float): distance to expand tree by at each step
    """

    def __init__(self, sampler, expand_dis=0.1, *args, **kwargs):
        super(RRTConnect, self).__init__()
        self.sampler = sampler
        self.expand_dis = expand_dis

    def expand_tree(self, node_list, target_state, env):
        """
        A function to expand the tree in the direction of target node
        """
        distance_list = [
            compute_distance(node.state.position, target_state.position)
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
        theta = np.arctan2(
            target_state.position.y - nearest_node.state.position.y,
            target_state.position.x - nearest_node.state.position.x,
        )
        new_point = Point()
        new_point.x = nearest_node.state.position.x + self.expand_dis * np.cos(theta)
        new_point.y = nearest_node.state.position.y + self.expand_dis * np.sin(theta)

        # Check whether new point is inside an obstacles
        traj = Trajectory(path=[nearest_node.state, RobotState(position=new_point)])

        new_node = Node.from_coordinates(new_point)
        new_node.parent = nearest_node

        if not env.get_traj_status(traj):
            valid = False
        else:
            valid = True

        return new_node, valid

    def plan(self, start, goal, env):
        """
        1. expand 1 tree. 
        2. make the new vertex target for the other tree. expand the 
           other tree until met an obstacle
        
        """
        # Check if start and goal states are obstacle free
        if not env.get_status(start):
            raise InvalidStartState(start, message="Start state is in obstacle.")

        if not env.get_status(goal):
            raise InvalidGoalState(goal, message="Goal state is in obstacle.")

        # Initialize start and goal nodes
        start_node = Node(state=start)
        goal_node = Node(state=goal)

        got_path = False

        # Initialize trajectory
        traj = Trajectory(path=[start, goal])

        # If the line between start and goal is obstacle free then connect directly
        # otherwise use rrt_connect
        if not env.get_traj_status(traj):
            node_list_from_start = [start_node]
            node_list_from_goal = [goal_node]
            tree_flag = True  # this flag is used to switch between the trees
            # if true expand from start node tree

            while True:
                rnd_state = self.sampler()
                # Randomly expand either of the trees
                if tree_flag:
                    # print("expanding from start")
                    new_node, valid = self.expand_tree(
                        node_list_from_start, rnd_state, env
                    )
                    if valid:
                        node_list_from_start.append(new_node)
                    else:
                        continue
                else:
                    # print("expanding from goal")
                    new_node, valid = self.expand_tree(
                        node_list_from_goal, rnd_state, env
                    )
                    if valid:
                        node_list_from_goal.append(new_node)
                    else:
                        continue

                # Use the new_node as the target for the other tree to expand
                if tree_flag:
                    # print("expanding towards goal")
                    while True:
                        new_node_target, valid = self.expand_tree(
                            node_list_from_goal, new_node.state, env
                        )
                        if not valid:
                            break
                        else:
                            node_list_from_goal.append(new_node_target)
                            # Check if target has been reached or if there is direct connection to target
                            distance_to_target = compute_distance(
                                new_node_target.state.position, new_node.state.position
                            )
                            traj = Trajectory(
                                path=[new_node_target.state, new_node.state]
                            )
                            if (
                                distance_to_target < self.expand_dis
                                or env.get_traj_status(traj)
                            ):
                                got_path = True
                                break
                else:
                    # print("expanding towards start")
                    while True:
                        new_node_target, valid = self.expand_tree(
                            node_list_from_start, new_node.state, env
                        )
                        if not valid:
                            break
                        else:
                            node_list_from_start.append(new_node_target)
                            # Check if target has been reached or if there is direct connection to target
                            distance_to_target = compute_distance(
                                new_node_target.state.position, new_node.state.position
                            )
                            traj = Trajectory(
                                path=[new_node_target.state, new_node.state]
                            )
                            if (
                                distance_to_target < self.expand_dis
                                or env.get_traj_status(traj)
                            ):
                                got_path = True
                                break

                if got_path:
                    break

                tree_flag = not tree_flag

            # Get the actual path from the
            path = []
            last_node = node_list_from_goal[-1]
            while last_node.parent is not None:
                path.append(last_node.state)
                last_node = last_node.parent
            path.append(goal)

            path = path[::-1]  # goal becomes the first element

            last_node = node_list_from_start[-1]
            while last_node.parent is not None:
                path.append(last_node.state)
                last_node = last_node.parent
            path.append(start)

            info_dict = {}
            info_dict["node_list"] = node_list_from_start + node_list_from_goal
            path = Trajectory(path[::-1])

            if len(path.path) == 1:
                raise PathNotFound(path, message="Path contains only one state")

            return (path, info_dict)

        else:
            goal_node.parent = start_node
            node_list = [start_node, goal_node]

            path = []
            last_node = node_list[-1]
            while last_node.parent is not None:
                path.append(last_node.state)
                last_node = last_node.parent
            path.append(start)

            info_dict = {}
            info_dict["node_list"] = node_list
            path = Trajectory(path[::-1])

            return (path, info_dict)
