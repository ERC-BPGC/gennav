from gennav.envs import PolygonEnv
from gennav.planners.potential_field import POTENTIAL_FIELD
from gennav.utils import RobotState
from gennav.utils.geometry import Point

# from gennav.envs.common import visualize_path


def test_potential_field():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]

    env = PolygonEnv()

    # add obstacles to the environment
    for obstacles in general_obstacles_list:
        env.update(obstacles)

        # Initialize the planner
        my_planner = potential_field(KP=5, ETA=100, THRESH=15, STEP_SIZE=0.1, error=0.2)
        start = RobotState(position=Point(0, 0))
        goal = RobotState(position=Point(10, 10))
        path = my_planner.plan(start, goal, env)

        # visualize_path(path, env)

        assert env.get_traj_status(path) is True
