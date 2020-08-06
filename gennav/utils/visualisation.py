from descartes import PolygonPatch
from matplotlib import pyplot as plt
from shapely.geometry import Polygon
from gennav.utils import RobotState


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
    chk=False
    for node in graph.nodes:
        if isinstance(node,RobotState):
            chk=True
            break
    if chk:
        for node in graph.nodes:
            for neighbour in graph.edges[node]:
                plt.plot(
                    [node.position.x, neighbour.position.x],
                    [node.position.y, neighbour.position.y],
                    color="red",
                )
    else:
        for node in graph.nodes:
            for neighbour in graph.edges[node]:
                plt.plot(
                    [node.state.position.x, neighbour.state.position.x],
                    [node.state.position.y, neighbour.state.position.y],
                    color="red",
                )

    # Draw the obstacles in the environment
    for obstacle in obstacle_list:
        obstacle_polygon = Polygon(obstacle)
        fig = plt.figure(1, figsize=(5, 5), dpi=90)
        ax = fig.add_subplot(111)
        poly_patch = PolygonPatch(obstacle_polygon)
        ax.add_patch(poly_patch)

    plt.show()
