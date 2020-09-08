from collections import defaultdict
from math import sqrt


class Graph:
    """Graph class used for graph based algorithms."""

    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, node):
        """Adds nodes to the graph.

        Args:
            node (gennav.utils.RobotState):to be added to the set of nodes.
        """
        self.nodes.add(node)

    def add_edge(
        self, node1, node2,
    ):
        """Adds edge connecting two nodes to the graph.

        Args:
            node1 (gennav.utils.RobotState): one end of the edge.
            node2 (gennav.utils.RobotState): other end of the edge.
        """
        self.edges[node1].append(node2)
        self.edges[node2].append(node1)
        self.distances[(node1, node2)] = self.calc_dist(node1, node2)

    def del_edge(self, node1, node2):
        """Deletes edge connecting two nodes to the graph.

        Args:
            node1 (gennav.utils.RobotState): one end of the edge.
            node2 (gennav.utils.RobotState): other end of the edge.
        """

        if len(self.edges[node1]) == 0:
            raise ValueError("Edge does not exist.")
        else:
            if len(self.edges[node1]) == 1:
                del self.edges[node1]
            else:
                self.edges[node1].remove(node2)

        del self.distances[(node1, node2)]

        if len(self.edges[node2]) == 0:
            raise ValueError("Edge does not exist.")
        else:
            if len(self.edges[node2]) == 1:
                del self.edges[node2]
            else:
                self.edges[node2].remove(node1)

    def calc_dist(self, node1, node2):
        """Calculates distance between two nodes.

        Args:
            node1 (gennav.utils.RobotState): one end of the edge.
            node2 (gennav.utils.RobotState): other end of the edge.

        Returns:
            dist (float): distance between two nodes.
        """
        self.dist = sqrt(
            (node1.position.x - node2.position.x) ** 2
            + (node1.position.y - node2.position.y) ** 2
            + (node1.position.z - node2.position.z) ** 2
        )
        return self.dist
