from collections import defaultdict

from gennav.utils import RobotState
from gennav.utils.common import Node


class Graph:
    """Graph class used for graph based algorithms.
    """

    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}

    def add_node(self, node=Node(state=RobotState())):
        """Adds nodes to the graph.

        Args:
            node (gennav.utils.common.Node):to be added to the set of nodes.
        """
        self.nodes.add(node)

    def add_edge(
        self,
        from_node=Node(state=RobotState()),
        to_node=Node(state=RobotState()),
        distance=0.0,
    ):
        """Adds edge connecting two nodes to the graph.

        Args:
            from_node (gennav.utils.common.Node): starting node of the edge.
            to_node (gennav.utils.common.Node): ending node of the edge.
            distance (float): The distance of the edge joining the two nodes.
        """
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance
