class Controller(object):
    """
        Base class for a controller.

        Args:
            init_state (gennav.utils.RoboState): Intial state of the robot.

        Attributes:
            robot_state (gennav.utils.RoboState): The state of the robot.

        TODO:
            Add actuation constraints
    """

    def __init__(self, init_state=None):
        self.robot_state = init_state

    def set_state(self, state):
        """ Set the state of the robot within the controller.

        Args:
            state (gennav.utils.RoboState): State to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        self.robot_state = state

    def compute_vel(self, traj):
        """ Compute the velocity according to given trajectory.

        Args:
            traj (gennav.utils.Trajectory): Trajectory to compute velocity for.

        Returns:
            gennav.utils.states.Velocity: The computed velocity.
        """
        raise NotImplementedError
