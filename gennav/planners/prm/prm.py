from gennav.planners.base import Planner
from gennav.utils import RobotState, Trajectory
from gennav.utils.geometry import compute_distance
from gennav.utils.graph import Graph
from gennav.utils.graph_search.astar import astar


class PRM(Planner):
    """PRM Class.

    Args:
        sampler (gennav.utils.sampler.Sampler): sampler to get random states
        r (float): maximum radius to look for neighbours
        n (int): total no. of nodes to be sampled in sample_area
    """

    def __init__(self, sampler, r, n):
        super(PRM, self)
        self.sampler = sampler
        self.r = r
        self.n = n

    def construct(self, env):
        """Constructs PRM graph.

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
            sample = self.sampler(self.sample_area)
            if not env.get_status(RobotState(position=sample)):
                continue
            else:
                i += 1
                nodes.append(sample)

        # finds neighbours for each node in a fixed radius r
        for node1 in nodes:
            for node2 in nodes:
                if node1 != node2:
                    dist = compute_distance(node1, node2)
                    if dist < self.r:
                        n1 = RobotState(position=node1)
                        n2 = RobotState(position=node2)
                        traj = Trajectory([n1, n2])
                        if env.get_traj_status(traj):
                            if n1 not in graph.nodes:
                                graph.add_node(n1)

                            if n2 not in graph.nodes:
                                graph.add_node(n2)

                            if n2 not in graph.edges[n1] and n1 not in graph.edges[n2]:
                                graph.add_edge(
                                    n1, n2,
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
        graph = self.construct(env)
        # find collision free point in graph closest to start_point
        min_dist = float("inf")
        for node in graph.nodes:
            dist = compute_distance(node.position, start.position)
            traj = Trajectory([node, start])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                s = node
        # find collision free point in graph closest to end_point
        min_dist = float("inf")
        for node in graph.nodes:
            dist = compute_distance(node.position, goal.position)
            traj = Trajectory([node, goal])
            if dist < min_dist and (env.get_traj_status(traj)):
                min_dist = dist
                e = node
        # add start_point to path
        path = [start]
        traj = Trajectory(path)
        # perform astar search
        p = astar(graph, s, e)
        if len(p.path) == 1:
            return traj
        else:
            traj.path.extend(p.path)
        # add end_point to path
        traj.path.append(goal)
        return traj
