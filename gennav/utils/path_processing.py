from gennav.utils.common import Trajectory, RobotState
from gennav.utils.geometry import Point, OrientationRPY
import numpy as np

def los_optimizer(traj, env):
    """
    Line of Sight Path Optimizer.

    For each point in the path, it checks if there is a direct
    connection to procceeding points which does not pass through
    any obstacles. By joining such points, number of uneccessary
    points in the path are reduced.

    Args:
        traj (gennav.utils.Trajectory): trajectory to optimize.
        env: (gennav.envs.Environment): environment to optimize path in.

    Returns:
        gennav.utils.Trajectory: Trajectory with otimized path.

        If path is found to be intersecting with any obstacle and
        there is no lookahead optimization which avoids this, then
        only the path uptill the intersection is returned.
    """
    path = traj.path

    # Init optimized path with the start as first point in path.
    optimized_path = [path[0]]

    # Loop through all points in path, checking for LOS shortening
    current_index = 0
    while current_index < len(path) - 1:

        # Keep track of whether index has been updated or not
        index_updated = False

        # Loop from last point in path to the current one, checking if
        # any direct connection exists.
        for lookahead_index in range(len(path) - 1, current_index, -1):
            if env.get_traj_status(
                Trajectory([path[current_index], path[lookahead_index]])
            ):
                # If direct connection exists then add this lookahead point to optimized
                # path directly and skip to it for next iteration of while loop
                optimized_path.append(path[lookahead_index])
                current_index = lookahead_index
                index_updated = True
                break

        # If index hasnt been updated means that there was no LOS shortening
        # and the edge between current and next point passes through an obstacle.
        if not index_updated:
            # In this case we return the path so far
            return optimized_path

    return optimized_path


        

        

        



    
        

