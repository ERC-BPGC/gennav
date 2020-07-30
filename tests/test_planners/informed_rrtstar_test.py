from gennav.envs import PolygonEnv
from gennav.planners.rrt.informed_rrtstar import InformedRRTstar
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler


def test_informedrrtstar():
    general_obstacles_list = [
        [(1, 1), (2, 1), (2, 2), (1, 2)],
        [(3, 4), (4, 4), (4, 5), (3, 5)],
    ]

    poly = PolygonEnv()
    poly.update(general_obstacles_list)
    sampler = UniformRectSampler(-2, 13, -2, 13)
    my_tree = InformedRRTstar(
        sampler=sampler,
        expand_dis=0.1,
        neighbourhood_radius=0.5,
        goal_distance=0.5,
        max_iter=2000,
    )
    start = RobotState(position=Point(1, 1))
    goal = RobotState(position=Point(10, 10))
    path = my_tree.plan(start, goal, poly)

    # from gennav.envs.common import visualize_path
    # visualize_path(path, poly)

    if len(path.path) != 1:
        assert poly.get_traj_status(path) is True
