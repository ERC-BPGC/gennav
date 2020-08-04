from gennav.envs import PolygonEnv
from gennav.planners.rrt.informed_rrtstar import InformedRRTstar
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler


def test_informedrrtstar():
    general_obstacles_list = [
        [(1, 1), (2, 1), (2, 2), (1, 2)],
        [(3, 4), (4, 4), (4, 5), (3, 5)],
        [(3, 2), (3, 3), (4, 3), (4, 2)],
    ]

    poly = PolygonEnv(buffer_dist=0.1)
    poly.update(general_obstacles_list)
    sampler = UniformRectSampler(0, 6, 0, 6)
    my_tree = InformedRRTstar(
        sampler=sampler,
        expand_dis=0.1,
        neighbourhood_radius=0.5,
        goal_distance=0.5,
        max_iter=2000,
    )
    start = RobotState(position=Point(0, 0))
    goal = RobotState(position=Point(6, 6))
    path, _ = my_tree.plan(start, goal, poly)

    # from gennav.utils.visualisation import visualize_node
    # node_list = _['node_list']
    # visualize_node(node_list, poly)

    # from gennav.envs.common import visualize_path
    # visualize_path(path, poly)

    if len(path.path) != 1:
        assert poly.get_traj_status(path) is True
