import random
import math
from shapely.geometry import Polygon, Point, LineString
from descartes import PolygonPatch
import matplotlib.pyplot as plt

from gennav.planners.rrtstar import RRTSTAR, random_sample_area, Node
from gennav.planners.samplers import uniform_adjustable_random_sampler as sampler
from gennav.utils.planner import visualize_path
from gennav.utils.planner import los_optimizer as path_optimizer


if __name__ == "__main__":
    general_obstacles_list = [
    [   [ (8, 5), (7, 8), (2, 9), (3, 5) ],
        [ (3, 3), (3, 5), (5, 5), (5, 3) ] ],
    [   [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
        [(4, 0), (4, 5), (5, 5), (5, 0)],
        [(8, 2), (8, 7), (10, 7), (10, 2)] ]
    ]
    
    for obstacles in general_obstacles_list:
        obstacle_list = obstacles

        # Instatiate rrt planner object
        my_tree = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)
        path, node_list = my_tree.plan((1, 1), (10, 10), obstacle_list)

        RRT.visualize_tree(node_list, obstacle_list)
        optimized_path = path_optimizer(path, obstacle_list)
        visualize_path(optimized_path, obstacle_list)
