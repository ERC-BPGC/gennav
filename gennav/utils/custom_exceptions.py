class Error(Exception):
    """Base class for exceptions in this module."""
    pass


class Key_Error(Error):
    """Exception raised when key is not found in the dictionary containing all parameters.

    Args:
        key (str): key to be searched in the dict.
        kwargs (dict): dictionary containing all input parameters
    """

    def __init__(self, key, kwargs):
        super().__init__()
        self.key = key
        self.dict = kwargs

    def __str__(self):
        try:
            return "%s key not found in the dictionary containing all parameters (dict_keys[%s])" % (self.key, list(self.dict.keys()))
        except SyntaxError:
            return f"{self.key} key not found in the dictionary containing all parameters ({self.dict.keys()})."


class StartStateinObs(Error):
    """Exception raised when starting state is inside an obstacle.

    Args:
        start (gennav.utils.RobotState): Start state
    """

    def __init__(self, start, message="Start state is in obstacle."):
        self.start = start
        self.message = message
        super(StartStateinObs, self).__init__()

    def __str__(self):
        try:
            return "%s -> %s" % (self.start, self.message)
        except SyntaxError:
            return f"{self.start} -> {self.message}"


class GoalStateinObs(Error):
    """Exception raised when ending state is inside an obstacle.

    Args:
        end (gennav.utils.RobotState): Goal state
    """

    def __init__(self, goal, message="Goal state is in obstacle."):
        self.end = goal
        self.message = message
        super(GoalStateinObs, self).__init__()

    def __str__(self):
        try:
            return "%s -> %s" % (self.end, self.message)
        except SyntaxError:
            return f"{self.end} -> {self.message}"


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
        try:
            return "%s is an invalid point -> %s" % (Self.sample, self.message)
        except SyntaxError:
            return f"{self.sample} is an invalid point -> {self.message}"


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
        try:
            return "%s -> %s" % (self.path, self.message)
        except SyntaxError:
            return f"{self.path} -> {self.message}"
