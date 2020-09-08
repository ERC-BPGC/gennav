import math

import numpy as np
from gennav.planners import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.common import Node
from gennav.utils.custom_exceptions import (
    InvalidGoalState,
    InvalidStartState,
    PathNotFound,
)
from gennav.utils.geometry import Point, compute_angle, compute_distance
from gennav.utils.samplers import UniformCircularSampler


class InformedRRTstar(Planner):
    def __init__(
        self,
        sampler,
        expand_dis=0.1,
        neighbourhood_radius=0.5,
        goal_distance=0.2,
        max_iter=500,
    ):
        """InformedRRT* Class

        Args:
            sampler (gennav.utils.sampler.Sampler): sampler to get random states
            expand_dis (float, optional): Distance by which the tree expands during the Steer operation . Defaults to 0.1.
            neighbourhood_radius (float, optional): Radius that determines the neighbours of a Node. Defaults to 0.5.
            goal_distance (float, optional): Distance which determines whether a node is near the goal or not. Defaults to 0.2.
            max_iter (int, optional): Maximum number of iterations so as to achieve the best possible path. Defaults to 500.
        """
        self.expand_dis = expand_dis
        self.neighbourhood_radius = neighbourhood_radius
        self.goal_distance = goal_distance
        self.max_iter = max_iter

        self.rectangular_sampler = sampler
        self.circular_sampler = UniformCircularSampler(1)

    def plan(self, start, goal, env):
        """Path planning method

        Args:
            start (gennav.utils.common.RobotState): Starting state.
            goal (gennav.utils.common.RobotState): Destination state
            env (gennav.envs.Environment): Environment object

        Returns:
            gennav.utils.Trajectory: The path as a Trajectory
            dict: Dictionary containing additional information like the node_list
        """
        # Check if start and goal states are obstacle free
        if not env.get_status(start):
            raise InvalidStartState(start, message="Start state is in obstacle.")

        if not env.get_status(goal):
            raise InvalidGoalState(goal, message="Goal state is in obstacle.")
        # Initialize start and goal nodes.
        x_start = Node(state=start)
        x_goal = Node(state=goal)
        # Starting the tree:
        node_list = [x_start]

        # X_soln contains all the nodes near the goal
        X_soln = []

        for i in range(self.max_iter):
            # defining the best cost as cbest
            if len(X_soln) == 0:
                cbest = float("inf")
            else:
                cbest = X_soln[0].cost + compute_distance(
                    X_soln[0].state.position, x_goal.state.position
                )
                for node in X_soln:
                    if (
                        node.cost
                        + compute_distance(node.state.position, x_goal.state.position)
                        < cbest
                    ):
                        cbest = node.cost + compute_distance(
                            node.state.position, x_goal.state.position
                        )
            # sample the new random point using the sample function
            if cbest < float("inf"):
                # Converting x_start and x_goal into arrays so that matrix calculations can be handled
                x_start_array = np.array(
                    [x_start.state.position.x, x_start.state.position.y],
                    dtype=np.float32,
                ).reshape(2, 1)

                x_goal_array = np.array(
                    [x_goal.state.position.x, x_goal.state.position.y], dtype=np.float32
                ).reshape(2, 1)

                cmin = np.linalg.norm((x_goal_array - x_start_array))

                x_center = (x_start_array + x_goal_array) / 2

                a1 = (x_goal_array - x_start_array) / cmin
                I1 = np.eye(len(x_start_array))[0].reshape(len(x_start_array), 1)

                M = np.dot(a1, I1.T)

                [u, s, v] = np.linalg.svd(M)

                # numpy returns the transposed value for v, so to convert it back
                v = v.T

                # Calculating the rotation to world frame matrix given by C = U*diag{1,....1.det(U)det(V)}*V.T where * denotes matrix multiplication
                diag = np.eye(2)
                diag[1, 1] = np.linalg.det(u) * np.linalg.det(v)
                C = np.dot(np.dot(u, diag), v.T)

                L = np.eye(2)
                L[0, 0] = cbest / 2
                L[1, 1] = np.sqrt(cbest ** 2 - cmin ** 2) / 2

                rand_state = self.circular_sampler()
                x_ball = np.array(
                    [rand_state.position.x, rand_state.position.y], dtype=np.float32
                ).reshape(2, 1)

                x_rand_array = np.dot(np.dot(C, L), x_ball) + x_center
                x_rand = RobotState()
                x_rand.position.x = x_rand_array[0, 0]
                x_rand.position.y = x_rand_array[1, 0]

                x_rand_node = Node(state=x_rand)

            else:
                x_rand = self.rectangular_sampler()
                x_rand_node = Node(state=x_rand)

            # Finding the nearest node to the random point
            distance_list = [
                compute_distance(node.state.position, x_rand_node.state.position)
                for node in node_list
            ]

            min_dist = distance_list[0]
            min_index = 0

            for index, distance in enumerate(distance_list):
                if distance < min_dist:
                    min_dist = distance
                    min_index = index

            x_nearest = node_list[min_index]

            # Steering at a distance of self.expand_dis

            theta = compute_angle(x_nearest.state.position, x_rand_node.state.position)

            x = x_nearest.state.position.x + self.expand_dis * math.cos(theta)
            y = x_nearest.state.position.y + self.expand_dis * math.sin(theta)
            t = RobotState(position=Point(x, y))

            x_new = Node(state=t)
            # Defining a temporary trajectory between the new node and its nearest node
            traj = Trajectory(path=[x_nearest.state, x_new.state])
            # Checking whether new node and the nearest node can be connected
            if env.get_traj_status(traj):
                # Adding new node to the node list
                node_list.append(x_new)

                # Defining the neighbours of x_new
                X_near = []
                for node in node_list:
                    if (compute_distance(node.state.position, x_new.state.position)) < (
                        self.neighbourhood_radius
                    ) and (  # If it is inside the neighbourhood radius
                        compute_distance(node.state.position, x_new.state.position) != 0
                    ):
                        X_near.append(node)

                x_min = x_nearest
                c_min = x_min.cost + self.expand_dis

                for x_near in X_near:
                    c_new = x_near.cost + compute_distance(
                        x_near.state.position, x_new.state.position
                    )

                    if c_new < c_min:
                        traj = Trajectory(path=[x_near.state, x_new.state])

                        if env.get_traj_status(traj):
                            x_min = x_near
                            c_min = c_new

                # Defining the parent node and cost of x_new
                x_new.parent = x_min
                x_new.cost = c_min

                # Rewiring
                for x_near in X_near:
                    c_near = x_near.cost
                    c_new = x_new.cost + compute_distance(
                        x_near.state.position, x_new.state.position
                    )

                    if c_new < c_near:
                        traj = Trajectory(path=[x_new.state, x_near.state])

                        if env.get_traj_status(traj):
                            x_near.parent = x_new
                            x_near.cost = c_new

                goal_traj = Trajectory(path=[x_new.state, x_goal.state])

                if (
                    compute_distance(x_goal.state.position, x_new.state.position)
                ) < self.goal_distance and env.get_traj_status(goal_traj):
                    X_soln.append(x_new)

        # X_soln contains all the solutions
        if len(X_soln) == 0:
            print("No solution found!")
            path = []
            path.append(x_start.state)
            traj = Trajectory(path=path)
            info_dict = {}
            info_dict["node_list"] = node_list
            if len(traj.path) == 1:
                raise PathNotFound(path, message="Path contains only one state")
            return (traj, info_dict)

        else:
            print("Goal reached!")
            Cbest = X_soln[0].cost
            Xbest = X_soln[0]
            node_list.append(x_goal)

            for node in X_soln:
                if node.cost < Cbest:
                    Cbest = node.cost
                    Xbest = node

            x_goal.parent = Xbest

            # Xbest is the optimum solution to the problem.

            # Back tracing the path:
            path = []
            path.append(x_goal.state)
            path.append(Xbest.state)
            p = Xbest.parent

            while p is not None:
                path.append(p.state)
                p = p.parent

            path.append(x_start.state)
            path.reverse()

            # print("No. of solutions " + str(len(X_soln)))

            trajectory = Trajectory(path=path)
            info_dict = {}
            info_dict["node_list"] = node_list

            return (trajectory, info_dict)

    def replan(self, start, goal, env):
        """Path replanning method

        Args:
            start (gennav.utils.common.RobotState): Starting state.
            goal (gennav.utils.common.RobotState): Destination state
            env (gennav.envs.Environment): Environment object

        Returns:
            gennav.utils.Trajectory: The path as a Trajectory
            dict: Dictionary containing additional information like the node_list
        """
        self.plan(start, goal, env)
