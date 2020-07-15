from gennav.envs import Environment, PolygonEnv
from gennav.planners.rrt.rrt import RRT
from gennav.utils import Trajectory, RobotState
from gennav.utils.path_processing import los_optimizer as path_optimizer
from gennav.utils.samplers import uniform_adjustable_random_sampler as sampler


def test_rrt():
    general_obstacles_list = [
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]

    poly = PolygonEnv(Environment)
    # poly.update(general_obstacles_list)

    for obstacles in general_obstacles_list:
        poly.update(obstacles)

        # Instatiate rrt planner object
        my_tree = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)
        path, node_list = my_tree.plan((1, 1), (10, 10), poly)

        # RRT.visualize_tree(node_list, obstacle_list)
        
        # Convert path (list of Nodes) to Tajectory (list of RobotStates) 
        for node in path:
            node = RobotState(position=node.state.position)
        path = Trajectory(path)
        
        optimized_path = path_optimizer(path, poly)
        # from gennav.envs.common import visualize_path
        # visualize_path(optimized_path, poly)

        assert poly.get_traj_status(optimized_path) is True
