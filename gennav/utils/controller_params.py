class PIDGains:
    """
        Class for storing Proportional Integral Controller Gains
        Args:
            kp : proportional gain (default = 0)
            ki : integral gain (default = 0)
            kd : derivative gain (default = 0)
    """

    def __init__(self, kp=0, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def __str__(self):
        return "PID :\nkp : {}\nki : {}\nkd : {}".format(self.kp, self.ki, self.kd)

    def __repr__(self):
        return "utils.common.PIDGains({}, {}, {})".format(self.kp, self.ki, self.kd)
