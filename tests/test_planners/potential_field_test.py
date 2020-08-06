from gennav.envs import PolygonEnv
from gennav.planners.potential_field import PotentialField
from gennav.utils import RobotState
from gennav.utils.geometry import Point


def test_potential_field():

    obstacle_list = [[(1, 1), (1.5, 1.5), (1.5, 1)]]
    env = PolygonEnv(buffer_dist=0.1)
    env.update(obstacle_list)
    my_planner = PotentialField(KP=5, ETA=100, THRESH=15, STEP_SIZE=0.1, error=0.2)
    start = RobotState(position=Point(0, 0))
    goal = RobotState(position=Point(7, 7))
    path, _ = my_planner.plan(start, goal, env)
    assert env.get_traj_status(path) is True
