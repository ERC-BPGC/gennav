class Planner(object):
    """
    Base class for path planning algorithms.

    """

    def plan(self, start, goal, env):
        """Plan path from start to goal in given environment.

        This method needs to be implemented in the specific path planner.

        Args:
            start (gennav.utils.common.RobotState): Initial robot state
            goal (gennav.utils.common.RobotState): Goal robot state
            env (gennav.envs.Env): Robot environment to plan in

        Returns:
            gennav.utils.common.Trajectory: The planned path as trajectory
        """
        raise NotImplementedError
