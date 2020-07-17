import random
from math import cos, pi, sin

from gennav.utils.geometry import Point


def uniform_adjustable_random_sampler(sample_area, goal, goal_sample_rate):
    """Randomly sample point in area while sampling goal point
        at a specified rate.
        Args:
            sample_area(tuple): area to sample point in (min and max)
            goal(gennav.utils.geometry.Point):Point representing goal point.
            goal_sample_rate: number between 0 and 1 specifying how often
                                to sample the goal point.

        Return:
            Randomly selected point as a Point(gennav.utils.geometry.Point).
    """

    if random.random() > goal_sample_rate:
        new_point = Point()
        new_point.x = random.uniform(sample_area[0], sample_area[1])
        new_point.y = random.uniform(sample_area[0], sample_area[1])
        return new_point
    else:
        return goal


def uniform_random_sampler(sample_area):
    """Randomly sample point in sample area
        Args:
            sample_area(tuple): area to sample point in (min and max)
        Return:
            Randomly selected point as a Point(gennav.utils.geometry.Point).
    """
    new_point = Point()
    new_point.x = random.uniform(sample_area[0], sample_area[1])
    new_point.y = random.uniform(sample_area[0], sample_area[1])
    return new_point


def uniform_random_circular_sampler(r):
    """Randomly samples point in a circular region of radius r around the origin.
        Args:
            r: radius of the circle.
        Return:
            Randomly selected point as Point(gennav.utils.geometry.Point).
    """
    # Generate a random angle theta.
    theta = 2 * pi * random.random()
    u = random.random() + random.random()
    if u > 1:
        a = 2 - u
    else:
        a = u

    new_point = Point()
    new_point.x = r * a * cos(theta)
    new_point.y = r * a * sin(theta)
    return new_point
