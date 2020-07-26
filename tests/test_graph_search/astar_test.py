
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.graph import Graph
from gennav.utils.graph_search.astar import astar


def test_astar():
    graph = Graph()
    node1 = RobotState(position=Point(0, 0))
    node2 = RobotState(position=Point(1, 1))
    node3 = RobotState(position=Point(3, 1))
    node4 = RobotState(position=Point(2, 2))
    node5 = RobotState(position=Point(3, 2))
    node6 = RobotState(position=Point(2, 3))

    graph.add_node(node1)
    graph.add_node(node2)
    graph.add_node(node3)
    graph.add_node(node4)
    graph.add_node(node5)
    graph.add_node(node6)

    graph.add_edge(node1, node2)
    graph.add_edge(node2, node1)
    graph.add_edge(node2, node3)
    graph.add_edge(node2, node4)
    graph.add_edge(node3, node1)
    graph.add_edge(node4, node1)
    graph.add_edge(node4, node5)
    graph.add_edge(node4, node6)
    graph.add_edge(node5, node4)
    graph.add_edge(node5, node6)
    graph.add_edge(node6, node4)
    graph.add_edge(node6, node5)

    start = Point(0, 0)
    end = Point(3, 2)
    _ = astar(graph, start, end)


def test_astar_heuristic():
    graph = Graph()
    node1 = RobotState(position=Point(0, 0))
    node2 = RobotState(position=Point(1, 1))
    node3 = RobotState(position=Point(3, 1))
    node4 = RobotState(position=Point(2, 2))
    node5 = RobotState(position=Point(3, 2))
    node6 = RobotState(position=Point(2, 3))

    graph.add_node(node1)
    graph.add_node(node2)
    graph.add_node(node3)
    graph.add_node(node4)
    graph.add_node(node5)
    graph.add_node(node6)

    graph.add_edge(node1, node2)
    graph.add_edge(node2, node1)
    graph.add_edge(node2, node3)
    graph.add_edge(node2, node4)
    graph.add_edge(node3, node1)
    graph.add_edge(node4, node1)
    graph.add_edge(node4, node5)
    graph.add_edge(node4, node6)
    graph.add_edge(node5, node4)
    graph.add_edge(node5, node6)
    graph.add_edge(node6, node4)
    graph.add_edge(node6, node5)
    heuristic = {
        Point(0, 0): 5,
        Point(1, 1): 3,
        Point(3, 1): 1,
        Point(2, 2): 1,
        Point(3, 2): 0,
        Point(2, 3): 2,
    }
    start = Point(0, 0)
    end = Point(3, 2)
    _ = astar(graph, start, end, heuristic)
