import math

try:
    from queue import PriorityQueue, Queue
except ImportError:
    from Queue import PriorityQueue, Queue


def astar(graph, start, end, heuristic={}):
    """Performs A-star search to find the shortest path from start to end

        Args:
            graph(dict): Dictionary representing the graph,
                    where keys are the nodes and the
                    value is a list of all neighbouring nodes
            start(tuple): Tuple representing key corresponding to the start point
            end(tuple): Tuple representing key corresponding to the end point
            heuristic(dict): Dictionary containing the heuristic values
                     for all the nodes, if not specified the default
                     heuristic is euclidean distance
        Returns:
            path(list):A list of points representing the path determined from
                    start to goal.An list containing just the start point means
                     path could not be planned.
    """
    open_ = PriorityQueue()
    closed = Queue()
    # calculates heuristic for start if not provided by the user
    # pushes the start point in the open_ Priority Queue
    if len(heuristic) == 0:
        open_.put(
            (math.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2), start)
        )
    else:
        open_.put((heuristic[start], start))
    # performs astar search to find the shortest path
    while len(open_.queue) > 0:
        current_node = open_.get()
        closed.put((current_node))
        # checks if the goal has been reached
        if current_node[1] == end:
            path = []
            while len(closed.queue) > 0:
                current_node = closed.get()
                path.append(current_node[1])
            return path
        # continues to search for the goal
        # makes a list of all neighbours of the current_node
        neighbours = graph[current_node[1]]
        # adds them to open_ if they are already present in open_
        # checks and updates the total cost for all the neighbours
        for node in neighbours:
            # calculates weight cost
            g = math.sqrt(
                (node[0] - current_node[1][0]) ** 2
                + (node[1] - current_node[1][0]) ** 2
            )
            # calculates heuristic for the node if not provided by the user
            if len(heuristic) == 0:
                h = math.sqrt((node[0] - end[0]) ** 2 + (node[1] - end[1]) ** 2)
            else:
                h = heuristic[current_node[1]][graph[current_node[1]].index(node)]
            # calculates total cost
            f = g + h
            # creates neighbour which can be pushed to open_ if required
            neighbour = (f, node)
            # checks if neighbour is in closed
            if neighbour in closed.queue:
                continue
            # checks if the total cost of neighbour needs to be updated
            # if it is presnt in open_ else adds it to open_
            for node in open_.queue:
                if neighbour == node and neighbour[0] > node[0]:
                    break
            open_.put(neighbour)
    # if path doesn't exsist it returns just the start point as the path
    path = [start]
    return path
