from gennav.envs import PolygonEnv
from gennav.planners.prm.prm import PRM
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler


def test_prm_plan():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]

    sampler = UniformRectSampler(-5, 15, -5, 15)
    poly = PolygonEnv(buffer_dist=0.1)
    start = RobotState(position=Point(0, 0))
    goal = RobotState(position=Point(12, 10))
    my_tree = PRM(sampler=sampler, r=5, n=100)

    for obstacles in general_obstacles_list:
        poly.update(obstacles)
        path, _ = my_tree.plan(start, goal, poly)

        from gennav.envs.common import visualize_path
        visualize_path(path, poly)

        if len(path.path) != 1:
            assert poly.get_traj_status(path) is True


def test_prm_construct():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]

    sampler = UniformRectSampler(-5, -5, 15, 15)
    poly = PolygonEnv()
    my_tree = PRM(sampler=sampler, r=5, n=50)

    for obstacles in general_obstacles_list:
        poly.update(obstacles)
        graph = my_tree.construct(poly)  # noqa: F841

        from gennav.utils.visualisation import visualize_graph
        visualize_graph(graph, poly)
test_prm_construct()