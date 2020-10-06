import math

import numpy as np
from gennav.utils.common import RobotState, Velocity
from matplotlib import pyplot as plt


class Simulator:
    """
    Simulator Class to simulate odometry for testing controllers
    Args:
           e = time period variable for each iteration of controller
    Returns:
           plot of iteration vs distance after running the controller for n iterations
           distance: array of distance from starting position with each iteration
    """

    def _init_(self, e=0.1):
        self.velocity = Velocity()
        self.e = e
        self.robot_state = RobotState()

    def simulator_omni(self, traj, n, controller):
        """
        it simulates odometry to test differential drive controller
        Args:
               traj: Trajectory point for the controller
               n: number of iterations controller should run
               controller: controller class that needs to be simulated
        """
        x_odom, y_odom = (0, 0)
        iterate = []
        distance = []
        i = 0
        while i <= n:
            self.robot_state.position.x = x_odom
            self.robot_state.position.y = y_odom
            self.velocity = controller.compute_vel(traj)
            self.velocity = controller.constrain(self.velocity)
            x_dist = self.velocity.linear.x * self.e
            y_dist = self.velocity.linear.y * self.e
            x_odom = x_odom + x_dist + np.random.rand()
            y_odom = y_odom + y_dist + np.random.rand()
            dist = math.sqrt(math.pow(x_odom, 2) + math.pow(y_odom, 2))
            distance = np.append(np.array(dist))
            iterate = np.append(np.array(i))
            i = i + 1
        plt.plot(iterate, distance)
        plt.xlabel("iteration")
        plt.ylabel("distance")
        plt.show()
        return distance

    def simulator_diff(self, traj, n, controller):
        """
        it simulates odometry to test a omnidrive controller
        Args:
               traj: trajectory point for the controller
               n: number of iterations the controller should run
               controller: controller class that needs to be simulated
        """
        x_odom, y_odom, yaw_odom = (0, 0, 0)
        distance = []
        iterate = []
        i = 0
        while i <= n:
            self.robot_state.position.x = x_odom
            self.robot_state.position.y = y_odom
            self.robot_state.orientation.yaw = yaw_odom
            self.velocity = controller.compute_vel(traj)
            self.velocity = controller.constrain(self.velocity)
            lin_dist = self.velocity.linear.x * self.e
            ang = self.velocity.angular.z * self.e
            ang = math.atan2(math.sin(ang), math.cos(ang))
            x_dist = lin_dist * math.cos(ang)
            y_dist = lin_dist * math.sin(ang)
            x_odom = x_odom + x_dist + np.random.rand()
            y_odom = y_odom + y_dist + np.random.rand()
            yaw_odom = yaw_odom + ang
            dist = math.sqrt(math.pow(x_odom, 2) + math.pow(y_odom, 2))
            distance = np.append(np.array(dist))
            iterate = np.append(np.array(i))
            i = i + 1
        plt.plot(iterate, distance)
        plt.xlabel("iteration")
        plt.ylabel("distance")
        plt.show()
        return distance
