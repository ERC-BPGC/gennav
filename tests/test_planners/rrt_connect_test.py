from gennav.envs import PolygonEnv
from gennav.planners.rrt.rrt_connect import RRTConnect
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler

# import timeit
# import time


def test_rrt_connect():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]
    sampler = UniformRectSampler(-5, 15, -5, 15)
    poly = PolygonEnv(buffer_dist=0.5)
    my_tree = RRTConnect(sampler=sampler, expand_dis=0.1)
    start = RobotState(position=Point(1, 1))
    goal = RobotState(position=Point(10, 10))

    for obstacles in general_obstacles_list:
        poly.update(obstacles)

        # start_time = timeit.default_timer()

        path, _ = my_tree.plan(start, goal, poly)

        # total_time = timeit.default_timer() - start_time
        # print("RRTConnect took {} s to build the tree".format(total_time))

        # from gennav.utils.visualisation import visualize_node
        # node_list = _["node_list"]
        # visualize_node(node_list, poly)
        # from gennav.envs.common import visualize_path
        # visualize_path(path, poly)

        assert poly.get_traj_status(path) is True


# if __name__ == "__main__":
#     test_rrt_connect()
