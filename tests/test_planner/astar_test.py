from gennav.planners.graph_search_algorithms.astar import astar


def test_astar():
    graph = {
        (0, 0): [(1, 1)],
        (1, 1): [(0, 0), (3, 1), (2, 2)],
        (3, 1): [(1, 1)],
        (2, 2): [(1, 1), (3, 2), (2, 3)],
        (3, 2): [(2, 2), (2, 3)],
        (2, 3): [(2, 2), (3, 2)],
    }
    start = (0, 0)
    end = (3, 2)
    _ = astar(graph, start, end)
