from descartes import PolygonPatch
from matplotlib import pyplot as plt
from shapely.geometry import Polygon


def visualize_graph(graph, env):
    """
    Draw the graph along with environment obstacles.

    Args:
        graph (dict): the dict representing the graph.
        env (gennav.envs.Environment): list of obtacles.
    """
    obstacle_list = env.obstacle_list
    # Clear the figure
    plt.clf()
    # Plot each edge of the tree
    for node in graph:
        for neighbour in graph[node]:
            plt.plot(
                [node.x, neighbour.x], [node.y, neighbour.y], color="red",
            )

    # Draw the obstacles in the environment
    for obstacle in obstacle_list:
        obstacle_polygon = Polygon(obstacle)
        fig = plt.figure(1, figsize=(5, 5), dpi=90)
        ax = fig.add_subplot(111)
        poly_patch = PolygonPatch(obstacle_polygon)
        ax.add_patch(poly_patch)

    plt.show()
