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


def split_path(traj, threshold):
    """Split straight line portions of the path into equal parts
        if larger than a threshold.
        For each line segment in the path, if the segment is above
        a threshold, points are inserted in equal distance, splitting
        it up into multiple segments.
        Args:
            traj: (gennav.utils.common.Trajectory): the trajectory to be split up
            threshold: length above which segments should be split up.
        Returns:
            gennav.utils.common.Trajectory : split up trajectory
    """
    path = traj.path
    i = 0
    while i <= len(path) - 1:
        vec1 = np.array([path[i].position.x, path[i].position.y, path[i].position.z])
        vec2 = np.array([path[i+1].position.x, path[i+1].position.y, path[i+1].position.z])
        if np.linalg.norm(vec2-vec1) > threshold:
            vec3 = vec1 + ((vec2 - vec1) * (threshold / np.linalg.norm(vec2 - vec1)))
            state = RobotState()
            state.position = Point(vec3[0], vec3[1], vec3[2])
            state.orientation = path[i].orientation
            path.insert(i+1, state)
        i += 1

    traj.path = path

    return traj

    def polygonFit(traj,env,deg=3, threshold=1, ret_vel_profile = False):
        """
            Fits a polyonomial of given degree to the trajectory taking into account 
            Args:
                traj (gennav.utils.common.Trajectory) : the trajectory to optimise
                env (gennav.utils.Environment) : to environment for collision checking
                deg (int default = 3) : the degree of polynomial to be fit into
                threshold(float default = 1) : threshold to break the control points
                ret_vel_profile (bool default=False) : returns a velocity profle according to the
                    timestamps in the trajcetory
            Returns:
                new trajectory (gennav.utils.common.trajectory)
        """

        X = [p.position.x for p in traj.path]
        y = [p.position.y for p in traj.path]

        poly = np.polynomial.polynomial.Polynomial.fit(X, y, deg)
        polyDer = poly.deriv(1)
        x = list(np.arange(X[0], X[-1], threshold))
        y = []
        yaws = []
        for x_ in x:
            y.append(poly(x_))
            yaws.append(polyDer(x_))
        rs = [RobotState(position=Point(x[i], y[i], 0), orientation=OrientationRPY(0, 0, yaws[i])) for i in range(len(x))]
        traj_new = Trajectory(path=rs)

        # TODO: Velocity Profile
        # Velocity Profile : Velocity profile might depend upon the controller
        # TODO: Collision checking and correction
        # Look at some good algos that can incorporate collision checking
        
        return traj_new

        

        

        



    
        

