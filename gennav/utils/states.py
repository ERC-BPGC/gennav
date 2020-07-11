from .geometry import OrientationRPY, Point, Vector3D


class RobotState:
    """
        Class for representing the robot state
        Args:
            position : class utils.common.Point (default = utils.common.Point)
            orientation : class utils.common.OrientationRPY (default = utils.common.OrientationRPY)
    """

    def __init__(self, position=Point(), orientation=OrientationRPY()):

        self.position = position
        self.orientation = orientation

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
        return "utils.common.RobotState(position=Point({}, {}, {}), orientation=OrientationRPY({}, {}, {}))".format(
            self.position.x,
            self.position.y,
            self.position.z,
            self.orientation.roll,
            self.orientation.pitch,
            self.orientation.yaw,
        )


class Velocity:
    """
        Class for representing Velocity
        Args:
            linear : class utils.common.Vector3D (default = utils.common.Vector3D)
            angular : class utils.common.Vector3D (default = utils.common.Vector3D)
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
        return "utils.common.Velocity(linear=utils.common.Vector3D({}, {}, {}), angular=utils.common.Vector3D({}, {}, {}))".format(
            self.linear.x,
            self.linear.y,
            self.linear.z,
            self.angular.x,
            self.angular.y,
            self.angular.z,
        )


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
