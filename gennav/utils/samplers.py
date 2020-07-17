import random
from gennav.utils.geometry import Point

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
        new_point=Point()
        new_point.x=random.uniform(sample_area[0], sample_area[1])
        new_point.y=random.uniform(sample_area[0], sample_area[1])
        return new_point
    else:
        goal_point=Point
        goal_point.x=goal[0]
        goal_point.y=goal[1]
        goal_point.z=goal[2]
        return goal_point


def uniform_random_sampler(sample_area):
    """Randomly sample point in sample area
        Args:
            sample_area: area to sample point in (min and max)
        Return:
            Randomly selected point as a tuple.
    """
    new_point=Point()
    new_point.x=random.uniform(sample_area[0], sample_area[1])
    new_point.y=random.uniform(sample_area[0], sample_area[1])  
    return new_point
