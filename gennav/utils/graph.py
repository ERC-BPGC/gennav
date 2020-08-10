from gennav.utils import RobotState
from .geometry import compute_distance


class Graph:
    """Graph class used for graph based algorithms.
    """

    def __init__(self):
        self.nodes = set()
        self.edges = {}
        self.distances = {}

    def add_node(self, node):
        """Adds nodes to the graph.

        Args:
            node (gennav.utils.RobotState):to be added to the set of nodes.
        """
        self.nodes.add(node)
        self.edges[node] = []

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
        if isinstance(node1, RobotState):
            self.distances[(node1, node2)] = compute_distance(
                node1.position, node2.position
            )
        else:
            self.distances[(node1, node2)] = compute_distance(
                node1.state.position, node2.state.position
            )

    def del_edge(self, node1, node2):
        """Deletes edge connecting two nodes to the graph.

        Args:
            node1 (gennav.utils.RobotState): one end of the edge.
            node2 (gennav.utils.RobotState): other end of the edge.
        """

        if node1 not in self.edges:
            raise ValueError("Edge does not exist.")
        else:
            if len(self.edges[node1]) == 1:
                del self.edges[node1]
            else:
                self.edges[node1].remove(node2)

        del self.distances[(node1, node2)]

        if node1 not in self.edges:
            raise ValueError("Edge does not exist.")
        else:
            if len(self.edges[node2]) == 1:
                del self.edges[node2]
            else:
                self.edges[node2].remove(node1)
