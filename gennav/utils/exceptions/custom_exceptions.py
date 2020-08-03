class Error(Exception):
    """Base class for exceptions in this module."""

    pass


class InvalidStartState(Error):
    """Exception raised when starting state is invalid.

    Args:
        start (gennav.utils.RobotState): Start state
    """

    def __init__(self, start, message="Start state is invalid."):
        self.start = start
        self.message = message
        super(InvalidStartState, self).__init__()

    def __str__(self):
        return "%s -> %s" % (self.start, self.message)


class InvalidGoalState(Error):
    """Exception raised when ending/goal state is invalid.

    Args:
        end (gennav.utils.RobotState): Goal state
    """

    def __init__(self, goal, message="Goal state is invalid."):
        self.end = goal
        self.message = message
        super(InvalidGoalState, self).__init__()

    def __str__(self):
        return "%s -> %s" % (self.end, self.message)


class SamplingFailed(Error):
    """Exception raised when the Sampler fails to return a valid state.

    Args:
        sample (gennav.utils.RobotState): state returned by the sampler
    """

    def __init__(self, sample, message="Sampler failed to return valid state."):
        self.sample = sample
        self.message = message
        super(SamplingFailed, self).__init__()

    def __str__(self):
        return "%s is an invalid point -> %s" % (self.sample, self.message)


class PathNotFound(Error):
    """Exception raised when path is not found.

    Args:
        path (gennav.utils.Trajectory): Path found by planner.
    """

    def __init__(self, path, message="Path could not be found."):
        self.path = path
        self.message = message
        super(PathNotFound, self).__init__()

    def __str__(self):
        return "%s -> %s" % (self.path, self.message)
