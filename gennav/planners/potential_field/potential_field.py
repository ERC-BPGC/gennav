import math

from gennav.planners.base import Planner
from gennav.utils import RobotState, Trajectory


class potential_field(Planner):
    """Potential Field Class

    Attributes:
        KP (float) : Attractive potential gain
        ETA (float) : Repulsive potential gain
        THRESH (float) : Distance beyond which attractive potential varies conically and repulsive potential has no effect
        STEP_SIZE (float) : Step size for gradient descent while add a waypoint
        error (float) : Minimum distance from the goal after which the bot will stop
    """

    def __init__(self, KP, ETA, THRESH, STEP_SIZE, error):
        """ initialise potential field parameters"""

        self.KP = KP
        self.ETA = ETA
        self.THRESH = THRESH
        self.error = error
        self.STEP_SIZE = STEP_SIZE

    def grad_attractive(self):
        """ Calculates the gradient due to the attractive component of the potential field, i.e, the potential field created by the goal point

        Args:
            None

        Returns:
            grad (float) : The gradient due to the attractive potential field at the state of the robot
        """

        dist = math.sqrt(
            (self.current.position.x - self.goal.position.x) ** 2
            + (self.current.position.y - self.goal.position.y) ** 2
        )
        if dist < self.THRESH:
            grad = [
                self.current.position.x - self.goal.position.x,
                self.current.position.y - self.goal.position.y,
            ]
            grad = [m * self.KP for m in grad]
        else:
            grad = [
                (self.current.position.x - self.goal.position.x) / dist,
                (self.current.position.y - self.goal.position.y) / dist,
            ]
            grad = [m * self.KP for m in grad]
        return grad

    def grad_repulsive(self):
        """Calculates the gradient due to the repulsive component of the potential field, i.e, the potential field created by the obstacles

        Args:
            None

        Returns:
            total_grad (float) : Total gradient due to potential field of all obstacles at the state of the robot
        """

        total_grad = [0, 0]
        min_dist = self.env.minimum_distances(self.current)
        for i, _ in enumerate(self.env.obstacle_list):
            if min_dist[i] < self.THRESH:
                dummy_state_x = RobotState(
                    (
                        self.current.position.x + 1,
                        self.current.position.y,
                        self.current.position.z,
                    )
                )
                x_grad = self.env.minimum_distances(dummy_state_x)[i]
                dummy_state_y = RobotState(
                    (
                        self.current.position.x,
                        self.current.position.y + 1,
                        self.current.position.z,
                    )
                )
                y_grad = self.env.minimum_distances(dummy_state_y)[i]
                grad = [
                    (x_grad - min_dist) / 0.01,
                    (y_grad - min_dist) / 0.01,
                ]
                factor = (
                    self.ETA
                    * ((1 / self.THRESH) - 1 / min_dist)
                    * 1
                    / (min_dist ** 2)
                )
                grad = [x * factor for x in grad]
                total_grad[0] = total_grad[0] + grad[0]
                total_grad[1] = total_grad[1] + grad[1]
        return total_grad

    def plan(self, start, goal, env):
        """ Plans the path to be taken by the robot to get from start to goal in the form of waypoints

        Args:
            start (gennav.utils.RobotState) : starting state of the robot (x, y and z coordinates)
            goal (gennav.utils.RobotState) : goal state of the robot (x, y and z coordinates)
            env (gennav.envs.Environment) : Type of environment the robot is operating in

        Returns:
            trajectory (gennav.utils.Trajectory) : A list of waypoints(in the form of robot states) that the robot will follow to go to the goal from the start
        """

        self.current = start
        self.goal = goal
        self.env = env
        waypoints = [self.current]
        while (
            math.sqrt(
                (self.current.position.x - self.goal.position.x) ** 2
                + (self.current.position.y - self.goal.position.y) ** 2
            )
            > self.error
        ):
            grad_attract = self.grad_attractive()
            grad_repel = self.grad_repulsive()
            grad = [0, 0]
            grad[0] = grad_attract[0] + grad_repel[0]
            grad[1] = grad_attract[1] + grad_repel[1]
            self.current.position.x = (
                self.current.position.x - self.STEP_SIZE * grad[0]
            )
            self.current.position.y = (
                self.current.position.y - self.STEP_SIZE * grad[1]
            )
            waypoints.append(self.current)
            trajectory = Trajectory(waypoints)
        return trajectory
