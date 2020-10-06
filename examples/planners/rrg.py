from gennav.envs import PolygonEnv
from gennav.envs.common import visualize_path  # module for visualizing path
from gennav.planners import RRG
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler

general_obstacles_list = [
    [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
    [
        [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
        [(4, 0), (4, 5), (5, 5), (5, 0)],
        [(8, 2), (8, 7), (10, 7), (10, 2)],
    ],
]

# A list of obstacles for the planner.

sampler = UniformRectSampler(-5, 15, -5, 15)  # for uniformly sampling points

poly = PolygonEnv(
    buffer_dist=1.0
)  # intializing the environment object needed for updating the obstacles of the environment

my_tree = RRG(
    sampler=sampler, expand_dis=1.0, max_iter=500
)  # initializing RRG planner object

start = RobotState(position=Point(0, 0))  # setting start point to (0,0)
goal = RobotState(position=Point(10, 10))  # setting end point to (0,0)

for obstacles in general_obstacles_list:  # updating the environment with all obstacles
    poly.update(obstacles)
path, _ = my_tree.plan(start, goal, poly)  # planning the path

visualize_path(path, poly)
