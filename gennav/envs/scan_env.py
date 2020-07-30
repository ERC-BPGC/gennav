#!/usr/bin/env python
import cmath
import math

import numpy as np
from matplotlib import pyplot as plt
from shapely.geometry import LineString

from .base import Environment


class ScanEnv(Environment):
    def __init__(self, scan_=None, bot_size=0.4, ang_range=[0, 2 * np.pi], viz=False):
        ScanEnv.__init__(self)
        self._scan = None
        self._bot_size = bot_size
        self._ang_range = ang_range
        self._pt_ang = None
        self._viz = viz

        self._escape_ind = []
        self._line_obstacle_ind = []
        self._line_obstacle = []

        if scan_ is not None:
            self._scan = scan_
            self._pt_ang = np.arange(
                self._ang_range[0],
                self._ang_range[1],
                (self._ang_range[1] - self._ang_range[0]) / len(scan_),
            )
            self._make_line_obstacle_ind()
            self._make_obstacles()

    def get_status(self, state):
        """ Get whether a given state is valid within the environment.

        Checks whether position of given state is clear of all points in the
        scan by a certain threshold given as the robot size.

        Args:
            state (gennav.utils.RoboState): State to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        point = state.position
        phi = math.atan2(point.x, point.y)
        rho = math.sqrt(point.x ** 2 + point.y ** 2)

        if self._viz:
            self._visualize(pt=point)

        for obstacle in zip(self._scan, self._pt_ang):
            # Checking the absolute of vector difference from each coordinate to be greater than THRESHOLD
            if (
                abs(
                    complex(cmath.rect(obstacle[0], obstacle[1]))
                    - complex(cmath.rect(rho, phi))
                )
                < self._bot_size
            ):
                return True
        return False

    def get_traj_status(self, traj):
        """ Get whether a given trajectory is valid within the environment.

        Checks if the path is clear of all obstacles by a certain threshold
        given as the robot size.

        Args:
            state (gennav.utils.Trajectory): Trajectory to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        path = traj.path
        path_coords = [(p.position.x, p.position.y) for p in path.poses]
        line_path = LineString(path_coords)

        if self._viz:
            self._visualize(pth=path_coords)

        for line_obst in self._line_obstacle:
            if line_path.distance(line_obst) < self._bot_size:
                return True

        return False

    def _make_line_obstacle_ind(self):
        """
        Make Line obstacle start and end index array from Laserscan
        """
        pt_scan = np.array([self._scan])

        # Shift Elements One index back in scan
        pt_scan_prev = np.append(pt_scan[0, 1:], pt_scan[0, 0])

        # Check for Areas from where Robot can escape
        escape_ind = abs(pt_scan - pt_scan_prev) > self._bot_size

        self._escape_ind = (np.argwhere(escape_ind[0] is True)).reshape(-1)

        for i in range(len(self._escape_ind)):

            ind_of_int = self._escape_ind[i]

            if np.isfinite(self._scan[ind_of_int]):
                prev = (
                    self._escape_ind[i - 1] + 1
                    if self._escape_ind[i - 1] + 1 < 360
                    else 0
                )
                self._line_obstacle_ind.append([prev, ind_of_int])

    def _make_obstacles(self):
        """
        Making Line obstacles from indexes
        """

        for i in self._line_obstacle_ind:
            line = self._get_points(i)
            if len(line) == 1:
                line = line + line
            self._line_obstacle.append(LineString(line))

    def _get_points(self, obst_st_en):
        if obst_st_en[0] <= obst_st_en[1]:

            return [
                (
                    self._scan[pt] * np.cos(self._pt_ang[pt]),
                    self._scan[pt] * np.sin(self._pt_ang[pt]),
                )
                for pt in range(obst_st_en[0], obst_st_en[1] + 1)
            ]

        else:

            l1 = [
                (
                    self._scan[pt] * np.cos(self._pt_ang[pt]),
                    self._scan[pt] * np.sin(self._pt_ang[pt]),
                )
                for pt in range(obst_st_en[0], len(self._scan))
            ]
            l2 = [
                (
                    self._scan[pt] * np.cos(self._pt_ang[pt]),
                    self._scan[pt] * np.sin(self._pt_ang[pt]),
                )
                for pt in range(0, obst_st_en[1] + 1)
            ]
            return l1 + l2

    def _visualize(self, pt=None, pth=None):
        plt.clf()
        pt_scan = np.array(self._scan)
        pts = []
        pt_x = np.multiply(pt_scan, np.cos(self._pt_ang))
        pt_y = np.multiply(pt_scan, np.sin(self._pt_ang))

        for a, b in zip(pt_x, pt_y):
            pts.append((a, b))

        plt.plot(pt_x, pt_y, "b.")

        for i in self._line_obstacle_ind:
            plt.plot(pt_x[i[0]], pt_y[i[0]], "kx")
            plt.plot(pt_x[i[1]], pt_y[i[1]], "kx")
        for i in self._line_obstacle:
            plt.plot(
                [x for (x, _) in list(i.coords)], [y for (_, y) in list(i.coords)], "r-"
            )

        if pt is not None:
            plt.plot(pt.x, pt.y, "kx")

        if pth is not None:
            for _ in pth:
                plt.plot([x for (x, _) in pth], [y for (_, y) in pth], "k-")

        plt.show()

    @property
    def scan(self):
        return self._scan

    def update(self, data):
        """ Update the environment.

        Updates internal representation with new scan data.

        Args:
            data: The updated scan data.
        """
        self._scan = data
        self._pt_ang = np.arange(
            self._ang_range[0],
            self._ang_range[1],
            (self._ang_range[1] - self._ang_range[0]) / len(self._scan),
        )
        self._make_line_obstacle_ind()
        self._make_obstacles()

        if self._viz:
            self._visualize()
