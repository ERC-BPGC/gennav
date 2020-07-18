import math

import matplotlib.patches as patches
import matplotlib.pyplot as plt
from gennav.utils.common import RobotState, Trajectory
from matplotlib.path import Path


class potential_field:
    def __init__(self, KP, ETA, THRESH, STEP_SIZE, error):
        self.KP = KP
        self.ETA = ETA
        self.THRESH = THRESH
        self.error = error
        self.STEP_SIZE = STEP_SIZE

    def grad_attractive(self):
        dist = math.sqrt(
            (self.current.position.x - self.goal.position.x) ** 2
            + (self.current.position.y - self.goal.position.y) ** 2
        )
        if dist < self.THRESH:
            grad = [
                self.current.position.x - self.goal.position.x,
                self.current.position.y - self.goal.position.y,
            ]
            grad = [m * self.KP for m in grad]
        else:
            grad = [
                (self.current.position.x - self.goal.position.x) / dist,
                (self.current.position.y - self.goal.position.y) / dist,
            ]
            grad = [m * self.KP for m in grad]
        return grad

    def grad_repulsive(self):
        total_grad = [0, 0]
        min_dist = self.env.minimum_distances(self.current)
        for i, obj in enumerate(self.env.obstacle_list):
            if min_dist[i] < self.THRESH:
                dummy_state_x = RobotState(
                    (
                        self.current.position.x + 1,
                        self.current.position.y,
                        self.current.position.z,
                    )
                )
                x_grad = self.env.minimum_distances(dummy_state_x)[i]
                dummy_state_y = RobotState(
                    (
                        self.current.position.x,
                        self.current.position.y + 1,
                        self.current.position.z,
                    )
                )
                y_grad = self.env.minimum_distances(dummy_state_y)[i]
                grad = [
                    (x_grad - min_dist) / 0.01,
                    (y_grad - min_dist) / 0.01,
                ]
                factor = (
                    self.ETA
                    * ((1 / self.THRESH) - 1 / min_dist)
                    * 1
                    / (min_dist ** 2)
                )
                grad = [x * factor for x in grad]
                total_grad[0] = total_grad[0] + grad[0]
                total_grad[1] = total_grad[1] + grad[1]
        return total_grad

    def plan(self, start, goal, env):
        self.current = start
        self.goal = goal
        self.env = env
        waypoints = [self.current]
        while (
            math.sqrt((self.x - self.x_g) ** 2 + (self.y - self.y_g) ** 2)
            > self.error
        ):
            grad_attract = self.grad_attractive()
            grad_repel = self.grad_repulsive()
            grad = [0, 0]
            grad[0] = grad_attract[0] + grad_repel[0]
            grad[1] = grad_attract[1] + grad_repel[1]
            self.current.position.x = (
                self.current.position.x - self.STEP_SIZE * grad[0]
            )
            self.current.position.y = (
                self.current.position.y - self.STEP_SIZE * grad[1]
            )
            waypoints.append(self.current)
            trajectory = Trajectory(waypoints)
        return trajectory

    def plot(self, trajectory):
        codes = []
        for i in range(len(trajectory.path)):
            codes.append(Path.LINETO)
        codes[0] = Path.MOVETO
        waypoints = []
        for obj in trajectory:
            waypoints.append((obj.position.x, obj.position.y))
        path = Path(waypoints, codes)
        fig, ax = plt.subplots()

        patch = patches.PathPatch(path, lw=2)
        ax.add_patch(patch)
        plt.xlim(-8, 8)
        plt.ylim(-8, 8)
        plt.show()
