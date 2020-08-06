from collections import defaultdict
from math import sqrt
from gennav.utils import RobotState


class Graph:
    """Graph class used for graph based algorithms.
    """

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
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        if isinstance(from_node,RobotState):
            self.distances[(from_node, to_node)] = self.calc_dist(from_node, to_node)
        else:
            self.distances[(from_node, to_node)] = self.calc_dist(from_node.state, to_node.state)
    def del_edge(self, from_node, to_node):
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
