from collections import defaultdict
from math import sqrt


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
        self, from_node, to_node,
    ):
        """Adds edge connecting two nodes to the graph.

        Args:
            from_node (gennav.utils.RobotState): starting node of the edge.
            to_node (gennav.utils.RobotState): ending node of the edge.
        """
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = self.calc_dist(from_node, to_node)

    def del_edge(self, from_node, to_node):
        """Deletes edge connecting two nodes to the graph.

        Args:
            from_node (gennav.utils.RobotState): starting node of the edge.
            to_node (gennav.utils.RobotState): ending node of the edge.
        """

        if len(self.edges[from_node]) == 0:
            raise ValueError("Edge does not exist.")
        else:
            if len(self.edges[from_node]) == 1:
                del self.edges[from_node]
            else:
                self.edges[from_node].remove(to_node)

        del self.distances[(from_node, to_node)]

        if len(self.edges[to_node]) == 0:
            raise ValueError("Edge does not exist.")
        else:
            if len(self.edges[to_node]) == 1:
                del self.edges[to_node]
            else:
                self.edges[to_node].remove(from_node)

    def calc_dist(self, from_node, to_node):
        """Calculates distance between two nodes.

        Args:
            from_node (gennav.utils.RobotState): starting node of the edge.
            to_node (gennav.utils.RobotState): ending node of the edge.

        Returns:
            dist (float): distance between two nodes.
        """
        self.dist = sqrt(
            (from_node.position.x - to_node.position.x) ** 2
            + (from_node.position.y - to_node.position.y) ** 2
            + (from_node.position.z - to_node.position.z) ** 2
        )
        return self.dist
