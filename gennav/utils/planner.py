# import random
import collections
import math

import matplotlib.pyplot as plt

import shapely
from descartes import PolygonPatch
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


def split_path(path, threshold):
    """Split straight line portions of the path into equal parts
        if larger than a threshold.
        For each line segment in the path, if the segment is above
        a threshold, points are inserted in equal distance, splitting
        it up into multiple segments.
        Args:
            path: list of tuples containing coordinates for a point in path..
            threshold: length above which segments should be split up.
        Returns:
            Split path as a list of tuples containing coordinates.
    """

    # split_path = []

    # for i in range(len(path) - 1):
    #     split_path.append(path[i])

    #     dist = math.sqrt((path[i][0] - path[i + 1][0])**2
    #                         + (path[i][1] - path[i + 1][1])**2)
    #     if dist > threshold:
    #         number_of_segments = dist // threshold
    #         for j in range(1, number_of_segments):


def los_optimizer(path, obstacle_list):
    """Line of Sight Path Optimizer.
        For each point in the path, it checks if there is a direct
        connection to procceeding points which does not pass through
        any obstacles. By joining such points, number of uneccessary
        points in the path are reduced.
        Args:
            path: list of tuples containing coordinates for a point in path..
            obstacle_list: list of obstacles.
        Returns:
            Optimized path as a list of tuples containing coordinates.
            If path is found to be intersecting with any obstacle and
            there is no lookahead optimization which avoids this, then
            only the path uptill the intersection is returned.
    """

    # Init optimized path with the start as first point in path.
    optimized_path = [path[0]]

    # Loop through all points in path, checking for LOS shortening
    current_index = 0
    while current_index < len(path) - 1:

        # Keep track of whether index has been updated or not
        index_updated = False

        # Loop from last point in path to the current one, checking if
        # any direct connection exists.
        for lookahead_index in range(len(path) - 1, current_index, -1):
            if not check_intersection(
                [path[current_index], path[lookahead_index]], obstacle_list
            ):
                # If direct connection exists then add this lookahead point to optimized
                # path directly and skip to it for next iteration of while loop
                optimized_path.append(path[lookahead_index])
                current_index = lookahead_index
                index_updated = True
                break

        # If index hasnt been updated means that there was no LOS shortening
        # and the edge between current and next point passes through an obstacle.
        if not index_updated:
            # In this case we return the path so far
            return optimized_path

    return optimized_path


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
