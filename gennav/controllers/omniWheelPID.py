from gennav.core import Controller
from gennav.utils.common import PIDGains, Velocity


class OmniWheelPID(Controller):
    """
        Controller class for an OmniWheel drive robot.
        It inherits from the main Controller class.
        Args:
            maxX : Maximum velocity in the x-direction (default = 0.25)
            maxY : Maximum velocity in the y-direction (default = 0.25)
            xgains : gennav.common.utils.PIDGains (default = PIDGains(1, 0, 0))
            ygains : gennav.common.utils.PIDGains (default = PIDGains(1, 0, 0))
        Returns:
            vel : (class utils.common.Velocity) with the required velocity commands
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
        self.xdiff, self.ydiff, self.xintegral, self.yintegral = 0, 0, 0, 0

    def move_bot(self, present, target):
        """
            Given present position and target position, returns velocity commands
            Args:
                present : class utils.common.Point
                target : class utils.common.Point
        """
        errorx = target.x - present.x
        errory = target.y - present.y

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

        velx = self.constrain(velx, "x")
        vely = self.constrain(vely, "y")

        self.velocity.linear.x = velx
        self.velocity.linear.y = vely

        self.xdiff = errorx - self.xdiff
        self.xintegral += errorx

        self.ydiff = errory - self.ydiff
        self.yintegral += errory

        return self.velocity

    def constrain(self, vel, dir):
        """
            Constrains the velocity within the given limits
            Args:
                vel : Velocity that needs to be constrained
                dir : Keyword for the velocity component
        """
        if dir.lower() == "x":
            velParam = self.maxX
        elif dir.lower() == "y":
            velParam = self.maxY
        else:
            raise Exception("Non recognised parameter {} passed.".format(dir))

        if vel > velParam:
            return velParam
        elif vel < -velParam:
            return -velParam
        else:
            return vel

    def parameters(self):
        return dict(
            {
                "xgains": self.xgains,
                "ygains": self.ygains,
                "maxX": self.maxX,
                "maxY": self.maxy,
                "xerrors": [self.xdiff, self.xintegral],
                "yerrors": [self.ydiff, self.yintegral],
            }
        )

    def restart(self):
        self.xdiff, self.xintegral, self.ydiff, self.yintegral = 0, 0, 0, 0
