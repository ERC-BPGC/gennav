from gennav.planners.prm.prm import PRM
from gennav.planners.samplers import uniform_random_sampler as sampler
from gennav.utils.planner import check_intersection
from gennav.utils.planner import los_optimizer as path_optimizer


def test_prm_plan():
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
        start = (0, 0)
        end = (12, 10)
        my_tree = PRM(sample_area=(-5, 15), sampler=sampler, r=5, n=100)
        path = my_tree.plan(start, end, obstacle_list)
        if len(path) > 1:
            optimized_path = path_optimizer(path, obstacle_list)
            # from gennav.utils.planner import visualize_path
            # visualize_path(optimized_path, obstacle_list)
            if len(optimized_path) > 1:
                assert check_intersection(optimized_path, obstacle_list) is False


def test_prm_construct():
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
        graph = my_tree.construct(obstacle_list)  # noqa: F841
        # PRM.visualize_graph(graph,obstacle_list)
