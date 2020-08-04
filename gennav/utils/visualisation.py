from descartes import PolygonPatch
from matplotlib import pyplot as plt
from shapely.geometry import Polygon


def visualize_graph(graph, env):
    """
    Draw the graph along with environment obstacles.

    Args:
        graph (gennav.utils.graph Graph): object of Graph class which stores the graph as a dict.
        env (gennav.envs.Environment): list of obtacles.
    """
    obstacle_list = env.obstacle_list
    # Clear the figure
    plt.clf()
    # Plot each edge of the tree
    for node in graph.nodes:
        for neighbour in graph.edges[node]:
            plt.plot(
                [node.position.x, neighbour.position.x],
                [node.position.y, neighbour.position.y],
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


def visualize_node(node_list, env):
    """
    Plot all the nodes and the obstacles in the environment

    Args:
        node_list (list): list containing all the nodes of the type (gennav.utils.common Node)
        env (gennav.envs.Environment): Environment object
    """
    obstacle_list = env.obstacle_list
    # Clear the figure
    plt.clf()
    # Plot all nodes
    for node in node_list:
        plt.scatter(node.state.position.x, node.state.position.y, c="k")

    for node in node_list:
        if node.parent is None:
            continue
        else:
            plt.plot(
                [node.parent.state.position.x, node.state.position.x],
                [node.parent.state.position.y, node.state.position.y],
                color="r",
            )

    # Draw the obstacles in the environment
    for obstacle in obstacle_list:
        obstacle_polygon = Polygon(obstacle)
        fig = plt.figure(1, figsize=(5, 5), dpi=90)
        ax = fig.add_subplot(111)
        poly_patch = PolygonPatch(obstacle_polygon)
        ax.add_patch(poly_patch)

    plt.show()
