"""
    Polygon Environment Class
"""

import shapely
from gennav.envs.base import Environment
from shapely.geometry import LineString, Polygon


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
        collision = False
        for obst in obstacles:
            if LineString(points).intersects(Polygon(obst)):
                collision = True
                break
        return collision

    def get_status(self, state):
        """Get whether a given state is valid within the given environment

        Checks whether the position given is not within any obstacle

        Args:
            state (gennav.utils.common.RobotState): State to be checked for

        Returns:
            bool : True if the state is valid, False otherwise
        """
        position = shapely.geometry.Point(
            state.position.x, state.position.y, state.position.z
        )
        valid = True
        for obst in self.obstacle_list:
            if position.within(Polygon(obst)):
                valid = False
                break
        return valid

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
        point = shapely.geometry.Point(
            state.position.x, state.position.y, state.position.z
        )
        nearest_obst = sorted(
            self.obstacle_list, key=lambda obj: point.distance(Polygon(obj))
        )[0]
        return point.distance(nearest_obst), nearest_obst

    def minimum_distances(self, state, sort=False):
        """Function to get the minimum distance of each obstacle form the robot state

        Uses shapely Point's distance method to obtain the minimum distances

        Args:
            state (gennav.utils.common.RobotState) : present state of the robot
            sort (bool default = False) : returns the distances in ascending form when set true
        Returns :
            min_dist (list) : list containing minimum distances(float) of each obstacle from the robot state
        """
        point = shapely.geometry.Point(
            state.position.x, state.position.y, state.position.z
        )
        min_dist = [point.distance(Polygon(obj)) for obj in self.obstacle_list]

        if sort:
            min_dist.sort()

        return min_dist
