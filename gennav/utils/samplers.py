import random
from math import cos, pi, sin

from gennav.utils.common import RobotState
from gennav.utils.geometry import Point


class Sampler(object):
    """Base classs for samplers.
    """

    def __call__(self):
        """Sample a configuration.

        This method needs to be implemented in the specific sampler.

        Returns:
            gennav.utils.RobotState: The sampled robot state.
        """
        raise NotImplementedError


class UniformRectSampler(Sampler):
    """Uniformly sample point in given rectangular area while sampling goal point with
        a specified probability.

    Args:
        min_x (float): Minimum x coordinate of sample area
        max_x (float): Maximum x coordinate of sample area
        min_y (float): Minimum y coordinate of sample area
        max_y (float): Maximum y coordinate of sample area
        goal (gennav.utils.RobotState): Goal state. Defaults to None
        goal_sample_p (float): Probability of sampling goal. Defaults to 0

    Raises:
        ValueError: If goal is not specified when goal_sample_p > 0
    """

    def __init__(self, min_x, max_x, min_y, max_y, goal=None, goal_sample_p=0):
        super(UniformRectSampler, self).__init__()
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.goal = goal
        self.goal_sample_p = goal_sample_p
        if goal is None and goal_sample_p > 0:
            raise ValueError("goal much be specified if goal_sample_p > 0")

    def __call__(self):
        """
        Randomly sample point in area while sampling goal point at a specified rate.

        Return:
            gennav.utils.RobotState: The sampled robot state.
        """
        if random.random() > self.goal_sample_p:
            new_point = Point()
            new_point.x = random.uniform(self.min_x, self.max_x)
            new_point.y = random.uniform(self.min_y, self.max_y)
            return RobotState(position=new_point)
        else:
            return self.goal


class UniformCircularSampler(Sampler):
    """Uniformly sample point in given circular area while sampling goal point with
        a specified probability.

    Args:
        radius (float): Radius of sample area
        centre (gennav.utils.geometry.Point): Centre of circle to sample from.
            Defaults to origin
        goal (gennav.utils.RobotState): Goal state. Defaults to None
        goal_sample_p (float): Probability of sampling goal. Defaults to 0

    Raises:
        ValueError: If goal is not specified when goal_sample_p > 0
    """

    def __init__(self, radius, centre=Point(), goal=None, goal_sample_p=0):
        super(UniformCircularSampler, self).__init__()
        self.centre = centre
        self.r = radius
        self.goal = goal
        self.goal_sample_p = goal_sample_p
        if goal is None and goal_sample_p > 0:
            raise ValueError("goal much be specified if goal_sample_p > 0")

    def __call__(self):
        """Randomly sample point in area while sampling goal point at a specified rate.

        Return:
            gennav.utils.RobotState: The sampled robot state.
        """
        if random.random() > self.goal_sample_p:
            theta = 2 * pi * random.random()
            u = random.random() * self.r
            new_point = Point()
            new_point.x = self.centre.x + u * cos(theta)
            new_point.y = self.centre.y + u * sin(theta)
            return RobotState(position=new_point)
        else:
            return self.goal
