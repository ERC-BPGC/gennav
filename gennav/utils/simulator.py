import math

import numpy as np
from matplotlib import pyplot as plt

from ...controllers.base import Controller
from .common import RobotState, Velocity


class simulator:
    """
    Simulator Class to simulate odometry for testing controllers
    Args:
           e = time period variable for each iteration of controller
    Returns:
           plot of iteration vs distance after running the controller for n iterations
    """

    def _init_(self, e=0.1):
        self.controller = Controller()
        self.velocity = Velocity()
        self.e = e
        self.x_odom, self.y_odom, self.yaw_odom = (0, 0, 0)
        self.dist = 0
        self.robot_state = RobotState()
        self.distance = []
        self.iter = []

    def simulator_omni(self, traj, n):
        """
        it simulates odometry to test differential drive controller
        Args:
               traj: Trajectory point for the controller
               n: number of iterations controller should run
        """

        i = 0
        while i <= n:
            self.robot_state.position.x = self.x_odom
            self.robot_state.position.y = self.y_odom
            self.velocity = self.controller.compute_vel(traj)
            self.velocity = self.controller.constrain(self.velocity)
            x_dist = self.velocity.linear.x * (self.e)
            y_dist = self.velcity.linear.y * (self.e)
            self.x_odom = self.x_odom + x_dist + np.random.rand()
            self.y_odom = self.y_odom + y_dist + np.random.rand()
            self.dist = math.sqrt(math.pow(self.x_odom, 2) + math.pow(self.y_odom, 2))
            self.distance = self.append(np.array(self.dist))
            self.iter.append(np.array(i))
            i = i + 1
        plt.plot(self.iter, self.distance)
        plt.xlabel("iteration")
        plt.ylabel("distance")
        plt.show()

    def simulator_diff(self, traj, n):
        """
        it simulates odometry to test a omnidrive controller
        Args:
               traj: trajectory point for the controller
               n: number of iterations the controller should run
        """

        i = 0
        while i <= n:
            self.robot_state.position.x = self.x_odom
            self.robot_state.position.y = self.y_odom
            self.robot_state.orientation.yaw = self.yaw_odom
            self.velocity = self.controller.compute_vel(traj)
            self.velocity = self.controller.constrain(self.velocity)
            lin_dist = self.velocity.linear.x * (self.e)
            ang = self.velocity.angular.z * (self.e)
            ang = math.atan2(math.sin(ang), math.cos(ang))
            x_dist = lin_dist * math.cos(ang)
            y_dist = lin_dist * math.sin(ang)
            self.x_odom = self.x_odom + x_dist + np.random.rand()
            self.y_odom = self.y_odom + y_dist + np.random.rand()
            self.yaw_odom = self.yaw_odom + ang
            self.dist = math.sqrt(math.pow(self.x_odom, 2) + math.pow(self.y_odom, 2))
            self.distance = self.append(np.array(self.dist))
            self.iter.append(np.array(i))
            i = i + 1
        plt.plot(self.iter, self.distance)
        plt.xlabel("iteration")
        plt.ylabel("distance")
        plt.show()
