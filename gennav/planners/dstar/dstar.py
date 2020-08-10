from gennav.planners.base import Planner
from gennav.utils import Trajectory
from gennav.utils.geometry import compute_distance
from gennav.utils.graph import Graph
from gennav.utils.common import Node
from gennav.utils.custom_exceptions import (
    InvalidGoalState,
    InvalidStartState,
    PathNotFound,
)


def c(node1, node2):
    if node1.t == float("inf") or node1.t == float("inf"):
        return float("inf")
    else:
        return compute_distance(node1.state.position, node2.state.position)


def insert(node, h_):
    global open_
    if node.tag == "NEW":
        node.k = h_
        node.h = h_
        node.tag = "OPEN"
        open_.append(node)
    if node not in open_:
        node.k = min(node.h, h_)
        node.h = h_
        closed.remove(node)
        node.tag = "OPEN"
        open_.append(node)
    else:
        node.k = min(node.k, h_)
        node.h = h_


class NodeDstar(Node):
    """
    Node class for Dstar Node
    """

    # Initialize the class
    def __init__(self, **data):
        Node.__init__(self, **data)
        self.h = None
        self.k = None
        self.t = None
        self.tag = "NEW"

    # Sort nodes
    def __lt__(self, other):
        return self.k < other.k

    def __le__(self, other):
        return self.k <= other.k

    def __ge__(self, other):
        return self.k >= other.k

    def __gt__(self, other):
        return self.k > other.k

    # Compare nodes
    def __eq__(self, other):
        if not isinstance(other, NodeDstar):
            return False
        return self.state.position == other.state.position

    def __hash__(self):
        return hash(self.state)


class DStar(Planner):
    """DStar Class.

    Args:
        sampler (gennav.utils.sampler.Sampler): sampler to get random states
        r (float): maximum radius to look for neighbours
        n (int): total no. of nodes to be sampled in sample_area
    """

    def __init__(self, sampler, r, n):
        super(DStar, self)
        self.sampler = sampler
        self.r = r
        self.n = n
        self.flag = 0

    def construct(self, env):
        """Constructs DStar graph.

        Args:
            env (gennav.envs.Environment): Base class for an envrionment.

        Returns:
            graph (gennav.utils.graph): A dict where the keys correspond to nodes and
                the values for each key is a list of the neighbour nodes
        """
        nodes = []
        graph = Graph()
        i = 0

        # samples points from the sample space until n points
        # outside obstacles are obtained
        while i < self.n:
            sample = self.sampler()
            if not env.get_status(sample):
                continue
            else:
                i += 1
                node = NodeDstar(state=sample)
                nodes.append(node)

        # finds neighbours for each node in a fixed radius r
        for node1 in nodes:
            for node2 in nodes:
                if node1 != node2:
                    dist = compute_distance(node1.state.position, node2.state.position)
                    if dist < self.r:
                        if env.get_traj_status(Trajectory([node1.state, node2.state])):
                            if node1 not in graph.nodes:
                                graph.add_node(node1)

                            if node2 not in graph.nodes:
                                graph.add_node(node2)

                            if (
                                node2 not in graph.edges[node1]
                                and node1 not in graph.edges[node2]
                            ):
                                graph.add_edge(
                                    node1, node2,
                                )

        return graph

    def plan(self, start, goal, env):
        """Constructs a graph avoiding obstacles and then plans path from start to goal within the graph.

        Args:
            start (gennav.utils.RobotState): tuple with start point coordinates.
            goal (gennav.utils.RobotState): tuple with end point coordinates.
            env (gennav.envs.Environment): Base class for an envrionment.
        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
            
        """
        # construct graph
        global graph, traj
        graph = self.construct(env)
        # find collision free point in graph closest to start_point
        min_dist = float("inf")
        for node in graph.nodes:
            dist = compute_distance(node.state.position, start.position)
            traj = Trajectory([node.state, start])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                start_node = node
        # find collision free point in graph closest to end_point
        min_dist = float("inf")
        for node in graph.nodes:
            dist = compute_distance(node.state.position, goal.position)
            traj = Trajectory([node.state, goal])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                goal_node = node
        global open_, closed
        open_ = []
        closed = []
        goal_node.h = 0
        goal_node.k = 0
        open_.append(goal_node)
        while len(open_) > 0:
            open_.sort()
            current_node = open_.pop(0)
            current_node.tag = "CLOSED"
            closed.append(current_node)
            if current_node.state.position == start_node.state.position:
                path = []
                path.append(start)
                # while current_node.parent is not None:
                #     print 1
                #     path.append(current_node.state)
                #     h_=float("inf")
                #     neighbours=graph.edges[current_node]
                #     for neighbour in neighbours:
                #         if neighbour.h<h_:
                #             node=neighbour
                #             h_=neighbour.h
                #     current_node=node
                while current_node != goal_node:
                    path.append(current_node.state)
                    current_node = current_node.parent
                path.append(goal_node.state)
                path.append(goal)
                traj = Trajectory(path)
                return traj
            neighbours = graph.edges[current_node]
            for neighbour in neighbours:
                if neighbour in closed:
                    continue
                neighbour.parent = current_node
                neighbour.h = compute_distance(
                    current_node.state.position, node.state.position
                )
                neighbour.k = neighbour.h
                neighbour.tag = "OPEN"
                open_.append(neighbour)
        path = [start]
        traj = Trajectory(path)
        raise PathNotFound(traj, message="Path contains only one state")

    def replan(self, start, goal, env):
        """Constructs a graph avoiding obstacles and then plans path from start to goal within the graph.

        Args:
            start (gennav.utils.RobotState): tuple with start point coordinates.
            goal (gennav.utils.RobotState): tuple with end point coordinates.
            env (gennav.envs.Environment): Base class for an envrionment.
        Returns:
            gennav.utils.Trajectory: The planned path as trajectory
            
        """
        global traj, open_, graph, closed
        min_dist = float("inf")
        old_traj = traj
        for node in old_traj.path:
            dist = compute_distance(node.position, start.position)
            traj_chk = Trajectory([node, start])
            if dist < min_dist and (env.get_traj_status(traj_chk)):
                min_dist = dist
                s = node
        a = old_traj.path.index(s)
        new_traj = Trajectory([start])
        new_traj.path.extend(old_traj.path[a:])
        if env.get_traj_status(new_traj):
            return new_traj
        print 1
        min_dist = float("inf")
        for node in graph.nodes:
            dist = compute_distance(node.state.position, start.position)
            traj = Trajectory([node.state, start])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                print 44
                start_node = node
        # find collision free point in graph closest to end_point
        min_dist = float("inf")
        for node in graph.nodes:
            dist = compute_distance(node.state.position, goal.position)
            traj = Trajectory([node.state, goal])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                print 55
                goal_node = node
        for state in traj.path:
            for item in graph.nodes:
                if item.state == state:
                    node = item
            if not env.get_status(state):
                node.t = float("inf")
                if node.tag != "OPEN":
                    print 9
                    if node.tag == "CLOSED":
                        closed.remove(node)
                    node.tag = "OPEN"
                    open_.append(node)
                for neighbour in graph.edges[node]:
                    if neighbour.tag != "OPEN":
                        print 9
                        if neighbour.tag == "CLOSED":
                            closed.remove(neighbour)
                        neighbour.tag = "OPEN"
                        open_.append(neighbour)
                break

            elif traj.path.index(state) < len(traj.path) - 1 and (
                not env.get_traj_status(
                    Trajectory([state, traj.path[traj.path.index(state) + 1]])
                )
            ):
                node.t = float("inf")
                if node.tag != "OPEN":
                    print 9
                    if node.tag == "CLOSED":
                        closed.remove(node)
                    node.tag = "OPEN"
                    open_.append(node)
                for neighbour in graph.edges[node]:
                    if neighbour.tag != "OPEN":
                        print 9
                        if neighbour.tag == "CLOSED":
                            closed.remove(neighbour)
                        neighbour.tag = "OPEN"
                        open_.append(neighbour)
                break
        while len(open_) > 0:
            open_.sort()
            current_node = open_.pop(0)
            current_node.tag = "CLOSED"
            closed.append(current_node)
            if current_node.k >= node.h and node.tag == "CLOSED":
                print 8
                current_node = start_node
                path = []
                path.append(start)
                # while current_node.parent is not None:
                #     print 1
                #     path.append(current_node.state)
                #     h_ = float("inf")
                #     neighbours = graph.edges[current_node]
                #     for neighbour in neighbours:
                #         if neighbour.h < h_:
                #             node_ = neighbour
                #             h_ = neighbour.h
                #     current_node = node_
                print (goal_node in graph.nodes)
                i = 0
                while current_node.parent is not None:
                    path.append(current_node.state)
                    current_node = current_node.parent
                    if i > 40:
                        traj = Trajectory(path)
                        print 22
                        return traj
                    i += 1
                path.append(goal_node.state)
                path.append(goal)
                traj = Trajectory(path)
                print 2
                return traj
            if current_node.k < current_node.h and current_node.k != None:
                print 4
                for neighbour in graph.edges[current_node]:
                    if (
                        neighbour.tag != "NEW"
                        and neighbour.h <= current_node.k
                        and current_node.h > neighbour.h + c(current_node, neighbour)
                    ):
                        current_node.parent = neighbour
                        current_node.h = neighbour.h + c(current_node, neighbour)
            if current_node.k == current_node.h and current_node.k != None:
                print 3
                for neighbour in graph.edges[current_node]:
                    if (
                        neighbour.tag == "NEW"
                        or (
                            neighbour.parent == current_node
                            and neighbour.h
                            != current_node.h + c(current_node, neighbour)
                        )
                        or (
                            neighbour.parent != current_node
                            and neighbour.h
                            > current_node.h + c(current_node, neighbour)
                        )
                    ):
                        neighbour.parent = current_node
                        insert(neighbour, current_node.h + c(current_node, neighbour))
            else:
                for neighbour in graph.edges[current_node]:
                    if neighbour.tag == "NEW" or (
                        neighbour.parent == current_node
                        and neighbour.h != current_node.h + c(current_node, neighbour)
                    ):
                        neighbour.parent = current_node
                        insert(neighbour, current_node.h + c(current_node, neighbour))
                    elif (
                        neighbour.parent != current_node
                        and neighbour.h > current_node.h + c(current_node, neighbour)
                    ):
                        insert(current_node, current_node.h)

                    elif (
                        neighbour.parent != current_node
                        and current_node.h > neighbour.h + c(current_node, neighbour)
                        and (neighbour.tag == "CLOSED")
                        and neighbour.h > current_node.k
                    ):
                        insert(neighbour, neighbour.h)

        path = [start]
        traj = Trajectory(path)
        raise PathNotFound(traj, message="Path contains only one state")
