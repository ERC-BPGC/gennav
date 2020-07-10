# import random
import collections
import math

import shapely
from descartes import PolygonPatch
from matplotlib import pyplot as plt
from shapely.geometry import LineString, Point, Polygon

from .transformations import euler_from_quaternion

# Usefull named tuple to use for storing Orientation
Orientation = collections.namedtuple("Orientation", ["roll", "pitch", "yaw"])


def check_intersection(points_list, obstacle_list):
    """Check whether line passes through any obstacle.

    Args:
        points_list: list of points in the line.
        obstacle_list: list of obstacles as list of points.
    Returns:
        boolean specifying whether or not the line intersects
        and of the obstacles.
    """
    direct_line = LineString(points_list)
    for obstacle in obstacle_list:
        if direct_line.intersects(Polygon(obstacle)):
            return True

    return False


def visualize_path(path, obstacle_list):
    """Draw the path along with environment obstacles.
        Args:
            path: list of points in the path as tuples.
            obstacle_list: list of obtacles.
        Returns:
            Nothing. Function is used to visualize path.
    """

    # Clear the figure
    plt.clf()

    # Plot each point in the path
    plt.plot([x for (x, _) in path], [y for (_, y) in path])

    # Draw the obstacles in the environment
    for obstacle in obstacle_list:
        obstacle_polygon = Polygon(obstacle)
        fig = plt.figure(1, figsize=(5, 5), dpi=90)
        ax = fig.add_subplot(111)
        poly_patch = PolygonPatch(obstacle_polygon)
        ax.add_patch(poly_patch)

    plt.show()


def transform(obj, position, orientation):
    """Tranform geometric object (shape, line, point etc)
        w.r.t given position and orientation in cartesian
        system of coordinates.
        Args:
            obj: shapely.geometry type object to be transformed.
            position: shapely.geometry.point denoting base location.
            orientation: utils.Orientation having base roll pitch and yaw.
        Returns:
            Transformed object of type shapely.geometry.
    """

    obj = shapely.affinity.translate(obj, -position.x, -position.y)
    obj = shapely.affinity.rotate(
        obj, angle=math.degrees(orientation.yaw), origin=(0, 0)
    )
    return obj


def unwrap_pose(pose):
    """Unwrap pose type ROS message into position and
        orientation as tuple.
        Args:
            pose: geometry_msgs/Pose
        Returns:
            position as Point and orientations as
            Orientation objects
    """

    position = Point(pose.position.x, pose.position.y)
    orientation = Orientation(
        euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
    )

    return position, orientation
