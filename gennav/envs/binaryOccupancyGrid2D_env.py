from gennav.envs.base import Environment
import numpy as np
import matplotlib.pyplot as plt
from gennav.utils.common import RobotState
from gennav.utils.geometry import Point

class BinaryOccupancyGrid2DEnv(Environment):

    def __init__(self, X = 10, Y = 10):
        super(BinaryOccupancyGrid2DEnv, self).__init__()
        self.X = X
        self.Y = Y
        self.scan = None
        self.robotPose = None
        self.grid = np.zeros((self.X, self.Y))

        # Storing transforms
        self.transforms = {}
        self.mapTbot = {"from" : "map",
            "to" : "bot",
            "transform" : np.array([[1, 0, self.X/2],[0, 1, self.Y/2],[0, 0, 1]]).reshape(3, 3)}
        self.botTworld = {"from" : "bot",
            "to" : "world",
            "transform" : np.empty((3, 3))}
        self.mapTworld = {"from" : "map", "to" : "world", "transform" : np.dot(self.mapTbot["transform"], self.botTworld["transform"])}
        self.transforms["mapTbot"] = self.mapTbot
        self.transforms["botTworld"] = self.botTworld
        self.transforms["mapTworld"] = self.mapTworld

    def update(self, scan, robotPose):
        self.scan = scan
        self.robotPose = robotPose
        self.fillOccupancy()
        self.compute_transforms()


    def fillOccupancy(self):
        """Function that fills the occupancy grid in every update
        
            Assumptions:
                1. RobotPose is (0, 0, 0) because the laser scan produces ranges wrt to the bot
                2. The RobotPose in the occupancy grid is (X/2, Y/2, 0)
                3. The attribut self.robotPose is the real pose of the robot wrt to the world Frame,
                    thus it helps us to calculate the transform for the queries 
        """
        self.grid[:] = 0
        ang_min, ang_max, ranges = self.scan
        angle_step = (ang_max - ang_min)/len(ranges)
        for i, rng in enumerate(ranges):

            # Check for obstacles 
            if rng is not np.inf:
                x, y = (self.X/2) + rng * np.cos(ang_min + i * angle_step), (self.Y/2) + rng * np.sin(ang_max + i * angle_step)

                # Checking if the range is within the grid, to mark them as occupied
                if 0 <= x < self.X and 0 <= y < self.Y:
                    if self.grid[int(x)][int(y)] != 1:
                        self.grid[int(x)][int(y)] = 1
            
    def get_status(self, state):
        pass
    def get_traj_status(self, traj):
        pass
    def nearest_obstacle_distance(self, state):
        pass

    def transform(self, frame1, frame2, rsf1):
        """Transform robotPose from one pose to the other

            Args:
                frame1 (string) : from the frame (world, bot, map)
                frame2 (string) : to the frame (world, bot, map)
                rsf1 (gennav.utils.common.RobotState) : RobotState in frame1
            Returns:
                (gennav.utils.common.RobotState) : RobotState in frame2
        """
        # TODO: Make it more robust in terms of checking frames

        # Check if the required trnasform or the inverse of the transform exists
        frame = frame2+ "T" + frame1
        frame_inv = frame1 + "T" + frame2
        if frame in self.transforms.keys():
            t_matrix = self.transforms[frame]["transform"]
        elif frame_inv in self.transforms.keys():
            t_matrix = np.linalg.inv(self.transforms[frame_inv]["transform"])
        else:
            raise Exception("Transform for the frames not found")
        
        # Transform using matrix multiplication
        pf2 = np.dot(t_matrix, np.array([rsf1.position.x, rsf1.position.y, 1]).reshape(-1, 1))
        rsf2 = RobotState(position=Point(pf2[0], pf2[1]))
        
        # Return RobotState
        return rsf2

    def compute_transforms(self):
        """Computes transforms between frames

            Uses robot pose to compute transform between the world frame and the bot frame
        """

        x, y, yaw = self.robotPose.position.x, self.robotPose.position.y, self.robotPose.orientation.yaw
        worldTbot = np.array([[np.cos(yaw), -np.sin(yaw), x],[np.sin(yaw), np.cos(yaw), y], [0, 0, 1]]).reshape(3, 3)
        self.botTworld["transform"] = np.linalg.inv(worldTbot)
        self.mapTworld["transform"] = np.dot(self.mapTbot["transform"], self.botTworld["transform"])
        

    def visualise_grid(self):
        fig, ax = plt.subplots()
        #ax.grid(color="black", alpha=0.5, ls = '-', lw=1)
        ax.imshow(self.grid, cmap="binary", vmin=0, vmax=1)
        plt.gca().invert_yaxis()
        plt.show()                

if __name__ == "__main__":
    
    ranges1 = [min(abs(np.random.uniform(10, 15)/np.cos(theta)), abs(np.random.uniform(10, 15)/np.sin(theta))) for theta in np.arange(0, 3.14*2, 3.14*2/100)]
    ranges2 = [np.random.uniform(20, 22) for _ in range(100)]
    ang_min = 0
    ang_max = 6.28
    obst = [[ang_min, ang_max, ranges1], [ang_min, ang_max, ranges2]]
    state = RobotState(Point(5, 5))
    
    env = BinaryOccupancyGrid2DEnv(50, 50)
    for ob in obst:
        env.update(ob, state)
        env.visualise_grid()

