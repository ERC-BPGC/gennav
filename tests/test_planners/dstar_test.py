from gennav.utils.graph import Graph
from gennav.planners.dstar.dstar import DStar
from gennav.envs import PolygonEnv
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler


def test_dstar():
    obstacles = [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]]

    sampler = UniformRectSampler(-5, 15, -5, 15)
    poly = PolygonEnv()
    start = RobotState(position=Point(0, 0))
    goal = RobotState(position=Point(12, 10))
    my_tree = DStar(sampler=sampler, r=3, n=75)

    poly.update(obstacles)
    path = my_tree.plan(start, goal, poly)
    from gennav.envs.common import visualize_path

    visualize_path(path, poly)
    obstacles = [
        [(8, 5), (7, 8), (2, 9), (3, 5)],
        [(3, 3), (3, 5), (5, 5), (5, 3)],
        [(10, 8), (12, 8), (11, 6)],
    ]

    # obstacles = [[(10, 7), (9, 10), (4, 11), (5, 7)], [(5, 5), (5, 7), (7, 7), (7, 5)],[(9,8),(13,8),(11,5)]]

    poly.update(obstacles)
    path_new = my_tree.replan(start, goal, poly)
    from gennav.envs.common import visualize_path

    visualize_path(path_new, poly)
    if len(path.path) != 1:
        assert poly.get_traj_status(path) is True


test_dstar()
