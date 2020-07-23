from gennav.planners.base import Planner
from gennav.planners.graph_search.astar import astar
from gennav.utils import RobotState, Trajectory
from gennav.utils.geometry import compute_distance


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
            graph (dict): A dict where the keys correspond to states and
                the values for each key is a list of the neighbour nodes
        """
        states = []
        graph = {}
        i = 0

        # samples points from the sample space until n points
        # outside obstacles are obtained
        while i < self.n:
            sample = self.sampler()
            if not env.get_status(sample):
                continue
            else:
                i += 1
                states.append(sample.position)

        # finds neighbours for each node in a fixed radius r
        for p1 in states:
            for p2 in states:
                if p1 != p2:
                    dist = compute_distance(p1, p2)
                    if dist < self.r:
                        traj = Trajectory(
                            path=[RobotState(position=p1), RobotState(position=p2)]
                        )
                        if env.get_traj_status(traj):
                            if p1 not in graph:
                                graph[p1] = [p2]
                            elif p2 not in graph[p1]:
                                graph[p1].append(p2)
                            if p2 not in graph:
                                graph[p2] = [p1]
                            elif p1 not in graph[p2]:
                                graph[p2].append(p1)

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

        # find collision free points in graph closest to start and goal
        min_dist_from_start = float("inf")
        min_dist_to_goal = float("inf")
        for point in graph.keys():
            dist_from_start = compute_distance(point, start.position)
            dist_to_goal = compute_distance(point, goal.position)
            state = RobotState(position=point)
            traj_from_start = Trajectory(path=[start, state])
            traj_to_goal = Trajectory(path=[state, goal])
            if dist_from_start < min_dist_from_start and (
                env.get_traj_status(traj_from_start)
            ):
                min_dist_from_start = dist_from_start
                closest_to_start = point
            if dist_to_goal < min_dist_to_goal and (env.get_traj_status(traj_to_goal)):
                min_dist_to_goal = dist_to_goal
                closest_to_goal = point

        # add start_point to path
        path = [start]
        traj = Trajectory(path)
        # perform astar search
        p = astar(graph, closest_to_start, closest_to_goal)
        if len(p.path) == 1:
            return traj
        else:
            traj.path.extend(p.path)
        # add end_point to path
        traj.path.append(goal)
        return traj
