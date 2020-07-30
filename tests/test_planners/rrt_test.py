from gennav.envs import PolygonEnv
from gennav.planners.rrt.rrt import RRT
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler


def test_rrt():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]
    sampler = UniformRectSampler(-5, 15, -5, 15)
    poly = PolygonEnv()
    my_tree = RRT(sampler=sampler, expand_dis=0.1)
    start = RobotState(position=Point(1, 1))
    goal = RobotState(position=Point(10, 10))

    for obstacles in general_obstacles_list:
        poly.update(obstacles)
        path = my_tree.plan(start, goal, poly)

        # from gennav.envs.common import visualize_path
        # visualize_path(path, poly)

        assert poly.get_traj_status(path) is True
