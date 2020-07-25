from gennav.envs import PolygonEnv
from gennav.planners.rrt.informed_rrtstar import InformedRRTstar
from gennav.utils import RobotState
from gennav.utils.geometry import Point


def test_informedrrtstar():
    general_obstacles_list = [
        [(1, 1), (2, 1), (2, 2), (1, 2)],
        [(3, 4), (4, 4), (4, 5), (3, 5)],
    ]

    poly = PolygonEnv()
    poly.update(general_obstacles_list)

    # Instantiate informed rrt* planner object
    my_tree = InformedRRTstar(
        sample_area=(-2, 13),
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

    assert poly.get_traj_status(path) is True
