from gennav.utils import Node, Trajectory
from gennav.utils.custom_exceptions import PathNotFound
from gennav.utils.geometry import compute_distance


class NodeAstar(Node):
    """
    Node class for Astar Node
    """

    # Initialize the class
    def __init__(self, **data):
        Node.__init__(self, **data)
        self.g = 0  # Distance to start node
        self.h = 0  # Distance to goal node
        self.f = 0  # Total cost

    # Sort nodes
    def __lt__(self, other):
        return self.f < other.f

    def __le__(self, other):
        return self.f <= other.f

    def __ge__(self, other):
        return self.f >= other.f

    def __gt__(self, other):
        return self.f > other.f

    # Compare nodes
    def __eq__(self, other):
        if not isinstance(other, NodeAstar):
            return False
        return self.state.position == other.state.position


def astar(graph, start, end, heuristic={}):
    """
    Performs A-star search to find the shortest path from start to end

    Args:
        graph (gennav.utils.graph): Dictionary representing the graph where keys are the nodes
            and the value is a list of all neighbouring nodes
        start (gennav.utils.RobotState): Point representing key corresponding
            to the start point
        end (gennav.utils.RobotState): Point representing key corresponding
            to the end point
        heuristic (dict): Dictionary containing the heuristic values for all the nodes,
            if not specified the default heuristic is euclidean distance

    Returns:
        gennav.utils.Trajectory: The planned path as trajectory
    """
    if not (start in graph.nodes and end in graph.nodes):
        path = [start]
        traj = Trajectory(path)
        return traj
    open_ = []
    closed = []
    # calculates heuristic for start if not provided by the user
    # pushes the start point in the open_ Priority Queue

    start_node = NodeAstar(state=start)
    if len(heuristic) == 0:
        start_node.h = compute_distance(start.position, end.position)
    else:
        start_node.h = heuristic[start]
    start_node.g = 0
    start_node.f = start_node.g + start_node.h
    open_.append(start_node)
    # performs astar search to find the shortest path
    while len(open_) > 0:
        open_.sort()
        current_node = open_.pop(0)
        closed.append(current_node)
        # checks if the goal has been reached
        if current_node.state.position == end.position:
            path = []
            # forms path from closed list
            while current_node.parent is not None:
                path.append(current_node.state)
                current_node = current_node.parent
            path.append(start_node.state)
            # returns reversed path
            path = path[::-1]
            traj = Trajectory(path)
            return traj
        # continues to search for the goal
        # makes a list of all neighbours of the current_node
        neighbours = graph.edges[current_node.state]
        # adds them to open_ if they are already present in open_
        # checks and updates the total cost for all the neighbours
        for node in neighbours:
            # creates neighbour which can be pushed to open_ if required
            neighbour = NodeAstar(state=node, parent=current_node)
            # checks if neighbour is in closed
            if neighbour in closed:
                continue
            # calculates weight cost
            neighbour.g = compute_distance(node.position, current_node.state.position)

            # calculates heuristic for the node if not provided by the user
            if len(heuristic) == 0:
                neighbour.h = compute_distance(node.position, end.position)
            else:
                neighbour.h = heuristic[node]

            # calculates total cost
            neighbour.f = neighbour.g + neighbour.h

            # checks if the total cost of neighbour needs to be updated
            # if it is presnt in open_ else adds it to open_
            flag = 1
            for new_node in open_:
                if neighbour == new_node and neighbour.f < new_node.f:
                    new_node = neighbour  # lgtm [py/multiple-definition]
                    flag = 0
                    break
                elif neighbour == new_node and neighbour.f > new_node.f:
                    flag = 0
                    break
            if flag == 1:
                open_.append(neighbour)

    # if path doesn't exsist it raises a PathNotFound Exception.
    path = [start]
    traj = Trajectory(path)
    raise PathNotFound(traj, message="Path contains only one state")
