from gennav.planners.graph_search_algorithms.astar import astar
import math

graph = {
    (0, 0): [(1, 1)],
    (1, 1): [(0, 0), (3, 1), (2, 2)],
    (3, 1): [(1, 1)],
    (2, 2): [(1, 1), (3, 2), (2, 3)],
    (3, 2): [(2, 2), (2, 3)],
    (2, 3): [(2, 2), (3, 2)],
}
weight = {
    (0, 0): [math.sqrt(2)],
    (1, 1): [math.sqrt(2), 4, math.sqrt(2)],
    (3, 1): [4],
    (2, 2): [math.sqrt(2), 1, 1],
    (3, 2): [1, math.sqrt(2)],
    (2, 3): [1, math.sqrt(2)],
}
start = (0, 0)
end = (3, 2)
path = []
path = astar(graph, weight, start, end)
