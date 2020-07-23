from gennav.utils.graph_search.astar import astar
from gennav.utils.geometry import Point


def test_astar():
    graph = {
        Point(0, 0): [Point(1, 1)],
        Point(1, 1): [Point(0, 0), Point(3, 1), Point(2, 2)],
        Point(3, 1): [Point(1, 1)],
        Point(2, 2): [Point(1, 1), Point(3, 2), Point(2, 3)],
        Point(3, 2): [Point(2, 2), Point(2, 3)],
        Point(2, 3): [Point(2, 2), Point(3, 2)],
    }
    start = Point(0, 0)
    end = Point(3, 2)
    _ = astar(graph, start, end)


def test_astar_heuristic():
    graph = {
        Point(0, 0): [Point(1, 1)],
        Point(1, 1): [Point(0, 0), Point(3, 1), Point(2, 2)],
        Point(3, 1): [Point(1, 1)],
        Point(2, 2): [Point(1, 1), Point(3, 2), Point(2, 3)],
        Point(3, 2): [Point(2, 2), Point(2, 3)],
        Point(2, 3): [Point(2, 2), Point(3, 2)],
    }
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
