from gennav.envs import PolygonEnv  # imports polygon environment
from gennav.envs.common import visualize_path
from gennav.planners.prm.prm import (
    PRM,  # imports Probabilistic Roadmap class from the planers
)
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler
from gennav.utils.visualisation import visualize_graph

# general_obstacle_list: obstacles in shapely defined by their vertices.
# sampler = UniformRectSampler(): samples nodes in some area (here: -5 < x < 15 and -5 < y < 15)
# r = maximum radius to look for neighbours
# n = total no. of nodes to be sampled in sample_area

# Using PRM to plan the path:


def prm_plan():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]

    sampler = UniformRectSampler(-5, 15, -5, 15)
    polygon = PolygonEnv()
    start = RobotState(position=Point(0, 0))
    goal = RobotState(position=Point(12, 10))

    my_prm = PRM(sampler=sampler, r=5, n=100)

    # Plan a path using the PRM.plan method for each obstacle:
    for obstacles in general_obstacles_list:
        polygon.update(obstacles)  # updates the environment with the obstacle
        path = my_prm.plan(start, goal, polygon)

        visualize_path(path, polygon)

        if len(path.path) != 1:  # check if the path has only the start state
            assert polygon.get_traj_status(path) is True


# Using PRM to just construct a graph:


def prm_construct():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]

    sampler = UniformRectSampler(-5, 15, -5, 15)
    polygon = PolygonEnv()
    my_prm = PRM(sampler=sampler, r=5, n=50)

    # Construct a graph in the configuration space of each obstacle using the PRM.construct method:
    for obstacles in general_obstacles_list:
        polygon.update(obstacles)  # updates the environment with the obstacle
        graph = my_prm.construct(polygon)  # noqa: F841

        visualize_graph(graph, polygon)
        
