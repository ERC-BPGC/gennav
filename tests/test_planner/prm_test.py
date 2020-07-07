import math

from gennav.planners.graph_search_algorithms.astar import astar
from gennav.planners.prm import PRM
from gennav.planners.samplers import uniform_random_sampler as sampler
from gennav.utils.planner import check_intersection
from gennav.utils.planner import los_optimizer as path_optimizer


def test_prm_astar():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]

    for obstacles in general_obstacles_list:
        obstacle_list = obstacles

        # Instatiate prm constructer object
        my_tree = PRM(sample_area=(-5, 15), sampler=sampler, r=5, n=50)
        graph = my_tree.construct(obstacle_list)
        # PRM.visualize_graph(graph,obstacle_list)
        start = (0, 0)
        end = (12, 10)

        min_dist = float("inf")
        for node in graph.keys():
            dist = math.sqrt((node[0] - start[0]) ** 2 + (node[1] - start[1]) ** 2)
            if dist < min_dist and (
                not check_intersection([node, start], obstacle_list)
            ):
                min_dist = dist
                s = node

        min_dist = float("inf")
        for node in graph.keys():
            dist = math.sqrt((node[0] - end[0]) ** 2 + (node[1] - end[1]) ** 2)
            if dist < min_dist and (not check_intersection([node, end], obstacle_list)):
                min_dist = dist
                e = node
        path = astar(graph, s, e)
        optimized_path = path_optimizer(path, obstacle_list)
        # from gennav.utils.planner import visualize_path
        # visualize_path(optimized_path, obstacle_list)

        assert check_intersection(optimized_path, obstacle_list) is False


def test_prm():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]

    for obstacles in general_obstacles_list:
        obstacle_list = obstacles

        # Instatiate prm constructer object
        my_tree = PRM(sample_area=(-5, 15), sampler=sampler, r=5, n=100)
        graph = my_tree.construct(obstacle_list)  # noqa: F841
        # PRM.visualize_graph(graph,obstacle_list)
