from matplotlib import pyplot as plt
from shapely.geometry import Polygon
from descartes import PolygonPatch


def visualize_path(traj, env):
    """
    Draw the path along with environment obstacles.

    Args:
        traj (gennav.utils.Trajectory): list of points in the path as tuples.
        env (gennav.envs.Environment): list of obtacles.
    """
    path = traj.path
    obstacle_list = env.obstacle_list
    # Clear the figure
    plt.clf()

    # Plot each point in the path
    plt.plot([p.position.x for p in path], [p.position.y for p in path], color="red")

    # Draw the obstacles in the environment
    for obstacle in obstacle_list:
        obstacle_polygon = Polygon(obstacle)
        fig = plt.figure(1, figsize=(5, 5), dpi=90)
        ax = fig.add_subplot(111)
        poly_patch = PolygonPatch(obstacle_polygon)
        ax.add_patch(poly_patch)

    plt.show()
