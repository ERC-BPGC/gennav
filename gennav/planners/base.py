class Planner:
    """
        Base class for path planning algorithms.

        TODO:
            Add state space constraints as an attribute.
    """

    def plan(self, start, goal, env):
        """Plan path from start to goal in given environment.

        This method needs to be implemented in the specific path planner.

        Args:
            start (gennav.utils.RoboState): Initial robot state
            goal (gennav.utils.RoboState): Goal robot state
            env (gennav.envs.Env): Robot environment to plan in

        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
        """
        raise NotImplementedError
