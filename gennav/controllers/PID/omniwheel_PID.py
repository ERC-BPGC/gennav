from ...utils.common import RobotState, Velocity
from ..base import Controller
from .common import PIDGains


class OmniWheelPID(Controller):
    """
        Controller class for an OmniWheel drive robot.
        It inherits from the main Controller class.
        Args:
            maxX : Maximum velocity in the x-direction (default = 0.25)
            maxY : Maximum velocity in the y-direction (default = 0.25)
            xgains : PIDGains (default = PIDGains(1, 0, 0))
            ygains : PIDGains (default = PIDGains(1, 0, 0))
        Returns:
            vel : (class utils.states.Velocity) with the required velocity commands
    """

    def __init__(
        self, maxX=0.25, maxY=0.25, xgains=PIDGains(1, 0, 0), ygains=PIDGains(1, 0, 0)
    ):
        super(OmniWheelPID, self).__init__()
        self.maxX, self.maxY = maxX, maxY
        self.xgains = xgains
        self.ygains = ygains

        # Initialise variables
        self.velocity = Velocity()
        self.robot_state = RobotState()
        self.xdiff, self.ydiff, self.xintegral, self.yintegral = 0, 0, 0, 0

    def compute_vel(self, traj):
        """
            Given the trajectory point, it returns the velocity using in differential format
            Args:
                traj : class gennav.utils.Trajectory : Trajectory to generate velocity
        """
        errorx = traj.x - self.robot_state.position.x
        errory = traj.y - self.robot_state.position.y

        velx = (
            self.xgains.kp * errorx
            + self.xgains.kd * self.xdiff
            + self.xgains.ki * self.xintegral
        )
        vely = (
            self.ygains.kp * errory
            + self.ygains.kd * self.ydiff
            + self.ygains.ki * self.yintegral
        )

        self.velocity.linear.x = velx
        self.velocity.linear.y = vely
        self.velocity = self.constrain(self.velocity)

        self.xdiff = errorx - self.xdiff
        self.xintegral += errorx

        self.ydiff = errory - self.ydiff
        self.yintegral += errory

        return self.velocity

    def constrain(self, velocity):
        """
            Constrains the velocity within the given limits
            Args:
                velocity : Velocity that needs to be constrained
        """
        if velocity.linear.x > self.maxX:
            velocity.linear.x = self.maxX
        elif velocity.linear.x < -self.maxX:
            velocity.linear.x = -self.maxX
        if velocity.linear.y > self.maxY:
            velocity.linear.y = self.maxY
        elif velocity.linear.y < -self.maxY:
            velocity.linear.y = -self.maxY

        return velocity

    def parameters(self):
        return dict(
            {
                "xgains": self.xgains,
                "ygains": self.ygains,
                "maxX": self.maxX,
                "maxY": self.maxY,
                "xerrors": [self.xdiff, self.xintegral],
                "yerrors": [self.ydiff, self.yintegral],
            }
        )

    def restart(self):
        self.xdiff, self.xintegral, self.ydiff, self.yintegral = 0, 0, 0, 0
