from .geometry import OrientationRPY, Point, Vector3D


class Velocity:
    """
        Class for representing Velocity

        Args:
            linear : class utils.geometry.Vector3D (default = utils.geometry.Vector3D)
            angular : class utils.geometry.Vector3D (default = utils.geometry.Vector3D)
    """

    def __init__(self, linear=Vector3D(), angular=Vector3D()):
        self.linear = linear
        self.angular = angular

    def __str__(self):
        return "Velocity : \n\tLinear : \n\t\tx : {}\n\t\ty : {}\n\t\tz : {}\n\tAngular : \n\t\tx : {}\n\t\ty : {}\n\t\tz : {}".format(
            self.linear.x,
            self.linear.y,
            self.linear.z,
            self.angular.x,
            self.angular.y,
            self.angular.z,
        )

    def __repr__(self):
        return "gennav.utils.common.Velocity(linear=utils.common.Vector3D({}, {}, {}), angular=utils.common.Vector3D({}, {}, {}))".format(
            self.linear.x,
            self.linear.y,
            self.linear.z,
            self.angular.x,
            self.angular.y,
            self.angular.z,
        )


class RobotState:
    """
        Class for representing the robot state

        Args:
            position : class utils.geometry.Point (default = utils.geometry.Point)
            orientation : class utils.geometry.OrientationRPY (default = utils.geometry.OrientationRPY)
            velocity : class utils.common.Velocity (default = utils.common.Velocity)
    """

    def __init__(
        self, position=Point(), orientation=OrientationRPY(), velocity=Velocity()
    ):
        self.position = position
        self.orientation = orientation
        self.velocity = velocity

    def __str__(self):
        return "RobotState :\n\tPosition :\n\tx : {}\n\ty : {}\n\tz : {}\n\tOrientation :\n\troll : {}\n\tpitch : {}\n\tyaw : {}\n\t".format(
            self.position.x,
            self.position.y,
            self.position.z,
            self.orientation.roll,
            self.orientation.pitch,
            self.orientation.yaw,
        )

    def __repr__(self):
        return "gennav.utils.RobotState(position=Point({}, {}, {}), orientation=OrientationRPY({}, {}, {}))".format(
            self.position.x,
            self.position.y,
            self.position.z,
            self.orientation.roll,
            self.orientation.pitch,
            self.orientation.yaw,
        )

    def __hash__(self):
        return hash(self.position)

    def __eq__(self, other):
        return self.position == other.position


class Trajectory:
    """
    Representation of robot trajectory as a series of robot states.

    Args:
        path (:obj:`list` of :obj:`gennav.utils.RobotState`): series of robot states
        timestamps (:obj:`list` of :obj:`datetime.datetime`, optional): timstamps for each state
    """

    def __init__(self, path, timestamps=None):
        self.path = path
        self.timestamps = timestamps


class Node:
    """Node class used in trees and graph representations.

    Args:
        data (dict): Arbitrary keyword arguments (**kwargs).
    """

    def __init__(self, **data):
        """Node init parameters.
        """
        self.data = data
        self.state = data.get("state", RobotState())
        self.parent = data.get("parent")
        self.cost = data.get("cost", 0.0)

    @classmethod
    def from_coordinates(cls, coordinates):
        """Create Node from coordinates.

        Args:
            coordinates (gennav.utils.geometry.Point): Point representing coordinates.

        Returns:
            gennav.utils.common.Node: state with the required conversion to Node.
        """
        return cls(state=RobotState(position=coordinates))

    @classmethod
    def from_orientation(cls, orientation_rpy):
        """Create Node from orientation (type of arg: OrientationRPY).

        Args:
            orientation_rpy (gennav.utils.geometry.OrientationRPY): OrientationRPY representing the roll, pitch and yaw.

        Returns:
            gennav.utils.common.Node: state with the required conversion to Node.
        """
        return cls(state=RobotState(orientation=orientation_rpy))
