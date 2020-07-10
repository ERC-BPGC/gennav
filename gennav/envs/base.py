class Environment(object):
    """
        Base class for an envrionment.

        An environment object should encapsulate all data processing
        related to a specific environment representation and should
        provide the ability to check for collision using this API
    """

    def get_status(self, state):
        """ Get whether a given state is valid within the environment.

        Args:
            state (gennav.utils.RoboState): State to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        raise NotImplementedError

    def get_trajectory_status(self, traj):
        """ Get whether a given trajectory is valid within the environment.

        Args:
            state (gennav.utils.Trajectory): Trajectory to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        raise NotImplementedError

    def update(self, *args, **kwargs):
        """ Update the environment.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        """
        raise NotImplementedError
