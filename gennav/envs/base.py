class Environment(object):
    """
        Base class for an envrionment.

        An environment object should encapsulate all data processing
        related to a specific environment representation and should
        provide the ability to check for collision using this API
    """

    def get_status(self, state):
        """ Get whether a given state is valid within the environment.

        This method needs to be implemented in the specific env implementation.

        Args:
            state (gennav.utils.RobotState): State to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        raise NotImplementedError

    def get_traj_status(self, traj):
        """ Get whether a given trajectory is valid within the environment.

        This method needs to be implemented in the specific env implementation.

        Args:
            state (gennav.utils.Trajectory): Trajectory to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        raise NotImplementedError

    def update(self, *args, **kwargs):
        """ Update the environment.

        This method needs to be implemented in the specific env implementation.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        """
        raise NotImplementedError
