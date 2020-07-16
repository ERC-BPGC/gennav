import random
from math import cos, pi, sin


def uniform_adjustable_random_sampler(sample_area, goal, goal_sample_rate):
    """Randomly sample point in area while sampling goal point
        at a specified rate.
        Args:
            sample_area: area to sample point in (min and max)
            goal: tuple containing goal point coordinates.
            goal_sample_rate: number between 0 and 1 specifying how often
                                to sample the goal point.

        Return:
            Randomly selected point as a tuple.
    """

    if random.random() > goal_sample_rate:
        return (
            random.uniform(sample_area[0], sample_area[1]),
            random.uniform(sample_area[0], sample_area[1]),
        )
    else:
        return goal


def uniform_random_sampler(sample_area):
    """Randomly sample point in sample area
        Args:
            sample_area: area to sample point in (min and max)
        Return:
            Randomly selected point as a tuple.
    """
    return (
        random.uniform(sample_area[0], sample_area[1]),
        random.uniform(sample_area[0], sample_area[1]),
    )


def uniform_random_circular_sampler(r):
    """Randomly samples point in a circular region of radius r around the origin.
        Args:
            r: radius of the circle.
        Return:
            Randomly selected point as tuple.
    """
    # Generate a random angle theta.
    theta = 2 * pi * random.random()
    u = random.random() + random.random()
    if u > 1:
        a = 2 - u
    else:
        a = u

    return (r * a * cos(theta), r * a * sin(theta))