"""
    Various common geometric constructs.
"""
import math

import numpy as np
import shapely


class Point:
    """
    A Basic Point class
    Args:
        x : x-coordinate of the point (default = 0)
        y : y-coordinate of the point (default = 0)
        z : z-coordinate of the point (default = 0)
    """

    def __init__(self, x=0, y=0, z=0):

        self.x = x
        self.y = y
        self.z = z

    def __hash__(self):
        return hash(self.x)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __str__(self):
        return "Point :\n\tx : {}\n\ty : {}\n\tz : {}".format(self.x, self.y, self.z)

    def __repr__(self):
        return "utils.geometry.Point({}, {}, {})".format(self.x, self.y, self.z)

    def __add__(self, val):
        if isinstance(val, int) or isinstance(val, float):
            return Point((self.x + val), (self.y + val), (self.z + val))
        elif isinstance(val, Point):
            return Point((self.x + val.x), (self.y + val.y), (self.z + val.z))
        else:
            raise TypeError(
                "Invalid datatype for addition. Only int, float and Point allowed"
            )

    def __sub__(self, val):
        if isinstance(val, int) or isinstance(val, float):
            return Point((self.x - val), (self.y - val), (self.z - val))
        elif isinstance(val, Point):
            return Point((self.x - val.x), (self.y - val.y), (self.z - val.z))
        else:
            raise TypeError(
                "Invalid datatype for subtraction. Only int, float and Point allowed"
            )

    def __mul__(self, val):
        if isinstance(val, int) or isinstance(val, float):
            return Point(self.x * val, self.y * val, self.z * val)
        elif isinstance(val, Point):
            return Point(self.x * val.x, self.y * val.y, self.z * val.z)
        else:
            raise TypeError(
                "Invalid datatype for multiplication. Only int, float and Point allowed"
            )

    def __rmul__(self, val):
        if isinstance(val, int) or isinstance(val, float):
            return Point(self.x * val, self.y * val, self.z * val)
        elif isinstance(val, Point):
            return Point(self.x * val.x, self.y * val.y, self.z * val.z)
        else:
            raise TypeError(
                "Invalid datatype for multiplication. Only int, float and Point allowed"
            )


class Quaternion:
    """
    Class for Quaternions
    Args :
        x : x-component of quaternion (default = 0)
        y : y-component of quaternion (default = 0)
        z : z-component of quaternion (default = 0)
        w : w-component of quaternion (default = 1)
    """

    def __init__(self, x=0, y=0, z=0, w=1):

        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __str__(self):
        return "Quaternion :\n\tx : {}\n\ty : {}\n\tz : {}\n\tw : {}".format(
            self.x, self.y, self.z, self.w
        )

    def __repr__(self):
        return "utils.geometry.Quaternion({}, {}, {}, {})".format(
            self.x, self.y, self.z, self.w
        )


class OrientationRPY:
    """
    Class for representing Orientation in terms if Roll, Pitch, Yaw
    Args:
        roll : rotation about x-axis (default = 0)
        pitch : rotation about y-axis (default = 0)
        yaw : rotation about z-axis (default = 0)
    """

    def __init__(self, roll=0, pitch=0, yaw=0):

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __str__(self):
        return "OrientaionRPY :\n\troll : {}\n\tpitch : {}\n\tyaw : {}".format(
            self.roll, self.pitch, self.yaw
        )

    def __repr__(self):
        return "utils.geometry.OrientaionRPY({}, {}, {})".format(
            self.roll, self.pitch, self.yaw
        )


class PointPath:
    """
    A class for storing Path
    Args:
        points = class utils.geometry.Point[] (a list of utils.geometry.Point)
    Note:
        This class does not take into consideration the orientation at each point
        For such a class use utils.geometry.Path
    """

    def __init__(self, points):
        self.points = points

    def __str__(self):
        return str(self.points)

    def __repr__(self):
        return str(self.points)


class PathComplete:
    """
    Class that can be used to represent path which contains RobotStates as waypoints
    Args:
        states = class utils.geometry.RobotState[] (a list of utils.geometry.RobotState)
    Note:
        For representing path using only Points, use utils.geometry.PointPath
    """

    def __init__(self, states):
        self.states = states

    def __str__(self):
        return str(self.states)

    def __repr__(self):
        return str(self.states)


class Vector3D:
    """
    Class for representing a 3 Dimensional Vector
    Args:
        x : x-component (default = 0)
        y : y-component (default = 0)
        z : z-component (default = 0)
    Note:
        This class contains additional methods wrt to the utils.geometry.Point Class
    """

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "Vector :\n\tx : {}\n\ty : {}\n\tz : {}".format(self.x, self.y, self.z)

    def __repr__(self):
        return "utils.geometry.Vector3D({}, {}, {})".format(self.x, self.y, self.z)

    def __add__(self, val):
        if isinstance(val, int) or isinstance(val, float):
            return Vector3D((self.x + val), (self.y + val), (self.z + val))
        elif isinstance(val, Vector3D):
            return Vector3D((self.x + val.x), (self.y + val.y), (self.z + val.z))
        else:
            raise TypeError(
                "Invalid datatype for addition. Only int, float and Vector3D allowed"
            )

    def __sub__(self, val):
        if isinstance(val, int) or isinstance(val, float):
            return Vector3D((self.x - val), (self.y - val), (self.z - val))
        elif isinstance(val, Vector3D):
            return Vector3D((self.x - val.x), (self.y - val.y), (self.z - val.z))
        else:
            raise TypeError(
                "Invalid datatype for subtraction. Only int, float and Vector3D allowed"
            )

    def __mul__(self, val):
        if isinstance(val, int) or isinstance(val, float):
            return Vector3D(self.x * val, self.y * val, self.z * val)
        elif isinstance(val, Vector3D):
            return Vector3D(self.x * val.x, self.y * val.y, self.z * val.z)
        else:
            raise TypeError(
                "Invalid datatype for multiplication. Only int, float and Vector3D allowed"
            )

    def __rmul__(self, val):
        if isinstance(val, int) or isinstance(val, float):
            return Vector3D(self.x * val, self.y * val, self.z * val)
        elif isinstance(val, Vector3D):
            return Vector3D(self.x * val.x, self.y * val.y, self.z * val.z)
        else:
            raise TypeError(
                "Invalid datatype for multiplication. Only int, float and Vector3D allowed"
            )

    def magnitude(self):
        """
        Returns the magnitude of the vector
        """
        return np.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def unit(self):
        """
        Returns the unit vector of the vector as a Vector3D object
        Returns:
            unitVector (utils.geometry.Vector3D) : Unit vector of the given vector
        """
        if self.x == 0 and self.y == 0 and self.z == 0:
            raise ZeroDivisionError("Cannot find unit Vector of a null vector")
        else:
            return Vector3D(
                self.x / self.magnitude(),
                self.y / self.magnitude(),
                self.z / self.magnitude(),
            )

    def directionCosines(self):
        """
        Returns the direction cosines of the vector, i.e cos(alpha), cos(beta), cos(gamma)
        """
        unit = self.unit()
        return unit.x, unit.y, unit.z


def transform(obj, position, orientation):
    """
    Tranform geometric object (shape, line, point etc) w.r.t given
    position and orientation in cartesian system of coordinates.
    Args:
        obj (shapely.geometry): object to be transformed.
        point (gennav.utils.geometry.Point): base location.
        orientation (gennav.utils.geometry.OrientationRPY): base roll pitch and yaw.
    Returns:
        gennav.utils.RoboState: Transformed robot state.
    """

    obj = shapely.affinity.translate(obj, -position.x, -position.y)
    obj = shapely.affinity.rotate(
        obj, angle=math.degrees(orientation.yaw), origin=(0, 0)
    )
    return obj


def transform_state(state, point, orientation):
    """
    Tranform a robot state w.r.t given position and orientation
    in cartesian system of coordinates.
    Args:
        state (gennav.utils.RoboState): state to be transformed.
        point (gennav.utils.geometry.Point): base location.
        orientation (gennav.utils.geometry.OrientationRPY): base roll pitch and yaw.
    Returns:
        gennav.utils.RoboState: Transformed robot state.
    """
    obj = shapely.geometry.Point(state.position.x, state.position.y, state.position.z)
    return transform(obj, point, orientation)


def transform_traj(traj, point, orientation):
    """
    Tranform a trajectory w.r.t given position and orientation
    in cartesian system of coordinates.
    Args:
        traj (gennav.utils.Trajectory): state to be transformed.
        point (gennav.utils.geometry.Point): base location.
        orientation (gennav.utils.geometry.OrientationRPY): base roll pitch and yaw.
    Returns:
        gennav.utils.RoboState: Transformed robot state.
    """
    obj = shapely.geometry.LineString(traj.path)
    return transform(obj, point, orientation)


def compute_distance(p1, p2):
    """Compute distance between two points.
    Args:
        p1 (gennav.utils.geometry.Point): One of the two points
        p2 (gennav.utils.geometry.Point): One of the two points
    Returns:
        float: The computed distance
    """
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)


def compute_angle(p1, p2):
    """Compute angle between two points.
    Args:
        p1 (gennav.utils.geometry.Point): One of the two points
        p2 (gennav.utils.geometry.Point): One of the two points
    Returns:
        float: The computed angle in radians
    """
    return math.atan2((p2.y - p1.y), (p2.x - p1.x))
