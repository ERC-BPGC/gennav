import random


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
