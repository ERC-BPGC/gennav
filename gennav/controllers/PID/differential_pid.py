import math

from ...utils.common import RobotState, Velocity
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
        self.robot_state = RobotState()
        self.dist_diff, self.angle_diff, self.dist_integral, self.angle_integral = (
            0,
            0,
            0,
            0,
        )

    def compute_vel(self, traj):
        """
            Given the trajectory point, it returns the velocity using in differential format
            Args:
                traj (gennav.utils.Trajectory): Trajectory to generate velocity
        """
        errorx = traj.x - self.robot_state.position.x
        errory = traj.y - self.robot_state.position.y
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

        self.velocity.linear.x = vel
        self.velocity.angular.z = (ang) - (self.robot_state.orientation.yaw)
        self.velocity = self.constrain(self.velocity)

        self.dist_diff = dist_error - self.dist_diff
        self.dist_integral += dist_error

        self.angle_diff = ang_error - self.angle_diff
        self.angle_integral += ang_error

        return self.velocity

    def constrain(self, velocity):
        """
            Constrains the velocity within the given limits
            Args:
                velocity (gennav.utils.states.velocity): Velocity that needs to be constrained
        """
        if velocity.linear.x > self.maxVel:
            velocity.linear.x = self.maxVel
        elif velocity.linear.x < -self.maxVel:
            velocity.linear.x = -self.maxVel
        if velocity.angular.z > self.maxAng:
            velocity.angular.z = self.maxAng
        elif velocity.angular.z < -self.maxAng:
            velocity.angular.z = -self.maxAng

        return velocity

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

    def reset(self):
        self.dist_diff, self.dist_integral, self.angle_diff, self.angle_integral = (
            0,
            0,
            0,
            0,
        )
