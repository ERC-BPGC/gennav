from gennav.utils.graph import Graph
from gennav.planners.dstar.dstar import DStar
from gennav.envs import PolygonEnv
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformRectSampler

def test_dstar():
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
    start = RobotState(position=Point(0, 0))
    goal = RobotState(position=Point(12, 10))
    my_tree = DStar(sampler=sampler, r=5, n=100)

    for obstacles in general_obstacles_list:
        poly.update(obstacles)
        # path = my_tree.plan(start, goal, poly)
        graph=my_tree.construct(poly)
        from gennav.utils.visualisation import visualize_graph
        visualize_graph(graph, poly)

        # if len(path.path) != 1:
        #     assert poly.get_traj_status(path) is True
test_dstar()