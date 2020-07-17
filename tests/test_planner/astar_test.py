from gennav.planners.graph_search.astar import astar
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
    print(_)


def test_astar_heuristic():
    graph = {
        (0, 0): [(1, 1)],
        (1, 1): [(0, 0), (3, 1), (2, 2)],
        (3, 1): [(1, 1)],
        (2, 2): [(1, 1), (3, 2), (2, 3)],
        (3, 2): [(2, 2), (2, 3)],
        (2, 3): [(2, 2), (3, 2)],
    }
    heuristic = {
        (0, 0): 5,
        (1, 1): 3,
        (3, 1): 1,
        (2, 2): 1,
        (3, 2): 0,
        (2, 3): 2,
    }
    start = Point(0, 0)
    end = Point(3, 2)
    _ = astar(graph, start, end, heuristic)
    print(_)
# test_astar_heuristic()
test_astar()