import math

from ...utils.robot_state import Velocity
from ..base import Controller
from .common import PIDGains


class DiffPID(Controller):
    """
        Controller class for an OmniWheel drive robot.
        It inherits from the main Controller class.
        Args:
            maxVel : Maximum velocity for translation(default = 0.25)
            maxAng : Maximum velocity for rotation(default = pi/2)
            vel_gains : PIDGains (default = PIDGains(1, 0, 0))
            angle_gains : PIDGains (default = PIDGains(1, 0, 0))
        Returns:
            vel : (class utils.states.Velocity) with the required velocity commands
    """

    def __init__(
        self,
        maxVel=0.25,
        maxAng=math.pi / 2,
        vel_gains=PIDGains(1, 0, 0),
        angle_gains=PIDGains(1, 0, 0),
    ):
        super(DiffPID, self).__init__()
        self.maxVel, self.maxAng = maxVel, maxAng
        self.vel_gains = vel_gains
        self.angle_gains = angle_gains

        # Initialise variables
        self.velocity = Velocity()
        self.dist_diff, self.angle_diff, self.dist_integral, self.angle_integral = (
            0,
            0,
            0,
            0,
        )

    def _move_bot(self, present, target, curr_orient):
        """
            Given present position and target position, returns velocity commands
            Args:
                present : class gennav.utils.geometry.Point
                target : class gennav.utils.geometry.Point
                curr_orient : class gennav.utils.geometry.OrientationRPY
        """
        errorx = target.x - present.x
        errory = target.y - present.y
        dist_error = math.sqrt(math.pow((errorx), 2) + math.pow((errory), 2))
        path_angle = math.atan2(errory, errorx)
        ang_error = math.atan2(math.sin(path_angle), math.cos(path_angle))

        vel = (
            self.vel_gains.kp * dist_error
            + self.vel_gains.kd * self.dist_diff
            + self.vel_gains.ki * self.dist_integral
        )

        ang = (
            self.angle_gains.kp * ang_error
            + self.angle_gains.kd * self.angle_diff
            + self.angle_gains.ki * self.angle_integral
        )

        vel = self.constrainV(vel)
        ang = self.constrainA(ang)

        self.velocity.linear.x = vel
        self.velocity.angular.z = (ang) - (curr_orient.yaw)

        self.dist_diff = dist_error - self.dist_diff
        self.dist_integral += dist_error

        self.angle_diff = ang_error - self.angle_diff
        self.angle_integral += ang_error

        return self.velocity

    def compute_vel(self, traj):
        """ Compute the velocity according to given trajectory.
        Args:
            traj (gennav.utils.Trajectory): Trajectory to compute velocity for.
        Returns:
            gennav.utils.states.Velocity: The computed velocity.
        # TODO #31
        """
        raise NotImplementedError

    def constrainV(self, vel):
        """
            Constrains the velocity within the given limits
            Args:
                vel : Velocity that needs to be constrained
        """
        if vel > self.maxVel:
            return self.maxVel
        elif vel < -self.maxVel:
            return -self.maxVel
        else:
            return vel

    def constrainA(self, ang):
        """
            Constrains the velocity within the given limits
            Args:
                ang : Angular velocity that needs to be constrained
        """
        if ang > self.maxAng:
            return self.maxAng
        elif ang < -self.maxAng:
            return -self.maxAng
        else:
            return ang

    def parameters(self):
        return dict(
            {
                "vel_gains": self.vel_gains,
                "angle_gains": self.angle_gains,
                "maxVel": self.maxVel,
                "maxAng": self.maxAng,
                "dist_errors": [self.dist_diff, self.dist_integral],
                "ang_errors": [self.angle_diff, self.angle_integral],
            }
        )

    def restart(self):
        self.dist_diff, self.dist_integral, self.angle_diff, self.angle_integral = (
            0,
            0,
            0,
            0,
        )
