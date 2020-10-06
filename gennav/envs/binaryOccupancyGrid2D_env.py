import numpy as np
from gennav.envs.base import Environment
from gennav.utils.common import RobotState
from gennav.utils.geometry import Point
from matplotlib import pyplot as plt


class BinaryOccupancyGrid2DEnv(Environment):
    """Base class for a Binary Occupancy Grid 2D envrionment.

    Arguments:
        X (unsigned int) : The number of grid cells in the x-direction
        Y (unsigned int) : the number of grid cells in the y-direction
    """

    def __init__(self, X=10, Y=10, *args, **kwargs):
        super(BinaryOccupancyGrid2DEnv, self).__init__()
        self.X = X
        self.Y = Y
        self.scan = None
        self.robotPose = None
        self.scale = 5
        self.grid = np.zeros((self.X * self.scale, self.Y * self.scale))

        # Storing transforms
        self.transforms = {}
        self.mapTbot = {
            "from": "map",
            "to": "bot",
            "transform": self.scale
            * np.array(
                [[1, 0, int(self.X / 2)], [0, 1, int(self.Y / 2)], [0, 0, 1]]
            ).reshape(3, 3),
        }
        self.botTworld = {"from": "bot", "to": "world", "transform": np.empty((3, 3))}
        self.mapTworld = {
            "from": "map",
            "to": "world",
            "transform": np.dot(self.mapTbot["transform"], self.botTworld["transform"]),
        }
        self.transforms["mapTbot"] = self.mapTbot
        self.transforms["botTworld"] = self.botTworld
        self.transforms["mapTworld"] = self.mapTworld

    def update(self, scan, robotPose):
        """Function to update the environment
        Args:
            scan (list) : List of ang_min, ang_max, ranges
            robotPose (gennav.utils.RobotPose) : Current RobotPose
        """
        self.scan = scan
        self.robotPose = robotPose
        self.compute_transforms()
        self.fillOccupancy()

    def fillOccupancy(self):
        """Function that fill the occupnacy grid on every update
        Assumptions:
                1. RobotPose is considered (0, 0, 0) to accomodate the laser scan, which produces ranges wrt to the bot
                2. The RobotPose in the occupancy grid is (X * scale_factor/2, Y * scale_factor /2, 0)
                3. The attribute robotPose is the real pose of the robot wrt to the world Frame,
                    thus it helps us to calculate the transform for trajectory and pose validity queries
        """
        self.grid[:] = 0
        ang_min, ang_max, ranges = self.scan
        angle_step = (ang_max - ang_min) / len(ranges)
        for i, rng in enumerate(ranges):

            # Check for obstacles
            if np.abs(rng) is not np.inf:
                x, y = (
                    rng * np.cos(ang_min + i * angle_step),
                    rng * np.sin(ang_max + i * angle_step),
                )
                newState = self.transform("bot", "map", RobotState(Point(x, y, 0)))
                x_, y_ = newState.position.x, newState.position.y

                # Checking if the range is within the grid, to mark them as occupied
                if 0 <= x_ < self.grid.shape[0] and 0 <= y_ < self.grid.shape[1]:
                    if self.grid[int(x_)][int(-y_ - 1)] != 1:
                        self.grid[int(x_)][int(-y_ - 1)] = 1

    def get_status(self, state):
        """Get whether a given state is valid within the environment.

        Method for checking the validity of a given RobotPose in the environment.

        Args:
            state (gennav.utils.RobotState): State to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        state = self.transform("world", "map", state)
        x, y = state.position.x, state.position.y
        if self.grid[x][-y - 1] == 1:
            return False
        else:
            return True

    def get_traj_status(self, traj):
        """Get whether a given trajectory is valid within the environment.

        Method for checking the validity of a trajectory in the given environment.

        Args:
            state (gennav.utils.Trajectory): Trajectory to be checked

        Returns:
            bool: True if state is valid otherwise False
        """
        collision = False
        for i in range(len(traj.path) - 1):
            collision = self.check_line_segment(
                self.transform("world", "map", traj.path[i]),
                self.transform("world", "map", traj.path[i + 1]),
            )
            if collision:
                break
        return not collision

    def transform(self, frame1, frame2, rsf1):
        """Transform robotPose from one pose to the other

        Args:
            frame1 (string) : from the frame (world, bot, map)
            frame2 (string) : to the frame (world, bot, map)
            rsf1 (gennav.utils.common.RobotState) : RobotState in frame1
        Returns:
            rsf2 (gennav.utils.common.RobotState) : RobotState in frame2
        """
        # TODO: Make it more robust in terms of checking frames

        # Check if the required trnasform or the inverse of the transform exists
        frame = frame2 + "T" + frame1
        frame_inv = frame1 + "T" + frame2
        if frame in self.transforms.keys():
            t_matrix = self.transforms[frame]["transform"]
        elif frame_inv in self.transforms.keys():
            t_matrix = np.linalg.inv(self.transforms[frame_inv]["transform"])
        else:
            raise Exception("Transform for the frames not found")

        # Transform using matrix multiplication
        pf2 = np.dot(
            t_matrix, np.array([rsf1.position.x, rsf1.position.y, 1]).reshape(3, 1)
        )
        rsf2 = RobotState(position=Point(pf2[0].item(), pf2[1].item()))

        # Return RobotState
        return rsf2

    def compute_transforms(self):
        """Computes transforms between frames

        Uses robot pose to compute transform between the world frame and the bot frame
        """
        x, y, yaw = (
            self.robotPose.position.x,
            self.robotPose.position.y,
            self.robotPose.orientation.yaw,
        )
        worldTbot = np.array(
            [[np.cos(yaw), -np.sin(yaw), x], [np.sin(yaw), np.cos(yaw), y], [0, 0, 1]]
        ).reshape(3, 3)
        self.botTworld["transform"] = np.linalg.inv(worldTbot)
        self.mapTworld["transform"] = np.dot(
            self.mapTbot["transform"], self.botTworld["transform"]
        )

    def visualise_grid(self):
        """
        Helper function to visualise grid
        """
        plt.imshow(self.grid, origin="bottom", cmap="binary")
        plt.show()

    def check_line_segment(self, state1, state2):
        """Checks whether a line segment is collision free in the environent

        Computes a line segment from the start point to the end point and
        parametrically checks if the grid cells they occupy are occupied.

        Args:
            state1 (gennav.utils.common.RobotState) : One end point
            state2 (gennav.utils.common.RobotState) : The other end point
        """
        point1 = state1.position
        point2 = state2.position
        x1, y1 = point1.x, point1.y
        x2, y2 = point2.x, point2.y
        m = (y2 - y1) / (x2 - x1)
        collision = False
        for x in np.arange(x1, x2, 0.5):
            y = m * x - m * x1 + y1
            if self.grid[int(x)][int(-y - 1)] == 1:
                collision = True
                break
        return collision
