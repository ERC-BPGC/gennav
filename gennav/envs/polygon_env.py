"""
    Polygon Environment Class
"""

from gennav.envs.base import Environment
from shapely.geometry import LineString, Point, Polygon


class PolygonEnv(Environment):
    """
        PolygonEnvironment Class
        Attributes:
            obstacles (list of Coordinates)
    """

    def __init__(self):
        super(PolygonEnv, self).__init__()
        self.obstacle_list = None

    def collision(self, points, obstacles):
        """
            A helper function that checks for Collision between the given obstacles and a set of points using shapely
            Args:
                points (list of coordinates (x, y)) - a list of points
                obstacles(list of list of cooridnates(x,y)) - a list of obstacles
            Returns:
                bool : True if collision is detected
        """
        for obst in obstacles:
            if LineString(points).intersects(Polygon(obst)):
                return True
        else:
            return False

    def get_status(self, state):
        """Get whether a given state is valid within the given environment

        Checks whether the position given is not within any obstacle

        Args:
            state (gennav.utils.common.RobotState): State to be checked for

        Returns:
            bool : True if the state is valid, False otherwise
        """
        position = Point(state.position.x, state.position.y, state.position.z)
        for obst in self.obstacle_list:
            if position.within(Polygon(obst)):
                return False
        else:
            return True

    def get_traj_status(self, traj):
        """Get whether a given trajectory is valid within the given environment

        Checks whether the given trajectory intersects with any obstacles or not

        Args:
            traj (gennav.utils.common.Trajectory): Trajectory to be checked for

        Returns:
            bool : True if the trajctory is valid, False otherwise
        """
        points = [(p.position.x, p.position.y) for p in traj.path]
        return not self.collision(points, self.obstacle_list)

    def update(self, obstacles):
        """Function call to update environment variables

        Updates the obstacles in the environment

        Args:
            obstacles : list of list of points (x, y) that make an obstacle
        Example of argument:
            [[(1, 1), (2, 1), (2, 2), (1, 2)],
             [(3, 4), (3.3, 4), (3.3, 4.2), (3, 4.2)]]
            The above list contains a list of points that constitute an obstacle. There can be arbitrary number of obstacles, which each obstacle having an arbitrary number of points
        """
        self.obstacle_list = obstacles

    def nearest_obstacle_distance(self, state, return_object=True):
        """Function to get the nearest obstacle to the robot state

        Uses shapely Point's distance method to obtain the distance

        Args:
            state (gennav.utils.common.RobotState) : present state of the robot
            return_object (bool default=True) : returns nearest obstacle also
        Returns:
            dist (float) : distance to the nearest obstacle
            obj (shapely.Polygon) : nearest obstacle

        """
        point = Point(state.position.x, state.position.y, state.position.z)
        nearest_obst = sorted(self.obstacle_list, key=lambda obj: point.distance(obj))[
            0
        ]
        return point.distance(nearest_obst), nearest_obst
