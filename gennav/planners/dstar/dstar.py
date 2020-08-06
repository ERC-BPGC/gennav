from gennav.planners.base import Planner
from gennav.utils import Trajectory
from gennav.utils.geometry import compute_distance
from gennav.utils.graph import Graph
from gennav.utils.graph_search.astar import astar
from gennav.planners.prm import PRM
from gennav.utils.common import Node

class NodeDstar(Node):
    """
    Node class for Dstar Node
    """

    # Initialize the class
    def __init__(self, **data):
        Node.__init__(self, **data)
        self.h = None 
        self.k = None 
        self.t= None
        self.tag="NEW"

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
        self.flag=0

    def construct(self,env):
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
                node=NodeDstar(state=sample)
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

    
    # def plan(self, start, goal, env):
    #     """Constructs a graph avoiding obstacles and then plans path from start to goal within the graph.

    #     Args:
    #         start (gennav.utils.RobotState): tuple with start point coordinates.
    #         goal (gennav.utils.RobotState): tuple with end point coordinates.
    #         env (gennav.envs.Environment): Base class for an envrionment.
    #     Returns:
    #         gennav.utils.Trajectory: The planned path as trajectory
            
    #     """
    #     if self.flag==0:
    #         flag=1
    #         # construct graph
    #         graph = self.construct(env)
    #         # find collision free point in graph closest to start_point
    #         min_dist = float("inf")
    #         for node in graph.nodes:
    #             dist = compute_distance(node.state.position, start.state.position)
    #             traj = Trajectory([node.state, start.state])
    #             if dist < min_dist and (env.get_traj_status(traj)):
    #                 min_dist = dist
    #                 start_node = node
    #         # find collision free point in graph closest to end_point
    #         min_dist = float("inf")
    #         for node in graph.nodes:
    #             dist = compute_distance(node.state.position, goal.state.position)
    #             traj = Trajectory([node.state, goal.state])
    #             if dist < min_dist and (env.get_traj_status(traj)):
    #                 min_dist = dist
    #                 goal_node = node
    #         open_=[]
    #         closed=[]
    #         goal_node.h=0
    #         goal_node.k=0
    #         open_.append(goal_node)
    #         while len(open_)>0:
    #             open_.sort()
    #             current_node = open_.pop(0)
    #             closed.append(current_node)   
    #             if current_node.state.position == start_node.state.position:
    #                 path = []
    #                 while current_node.parent is not None:
    #                     path.append(current_node.state)
    #                     current_node=current_node.parent
    #                 path.append(goal_node.state)
    #                 traj = Trajectory(path)
    #                 return traj             
    #             neighbours=graph.edges[current_node]
    #             for neighbour in neighbours:
    #                 neighbour.parent=current_node
    #                 if neighbour in closed:
    #                     continue
    #                 neighbour.h=compute_distance(current_node.state.position,node.position)
    #                 neighbour.k=neighbour.h
    #                 nodes.append(goal_node)
    #                 open_.append(neighbour)
                    

    #     else:
    #         min_dist = float("inf")
    #         old_traj=traj
    #         for node in old_traj.path:
    #             dist = compute_distance(node.position, start.position)
    #             traj_chk = Trajectory([node, goal])
    #             if dist < min_dist and (env.get_traj_status(traj_chk)):
    #                 min_dist = dist
    #                 s = node
    #         a=old_traj.path.index(s)
    #         old_traj.path=old_traj.path[a:]
    #         if env.get_traj_status(old_traj):
    #             return old_traj
    #         for state in traj.path:
    #             if not get_status(state):
    #                 for node in nodes:
    #                     if node.state=state:
    #                         node.t=float('inf')
    #                         break
    #                 if node not in open_:
    #                     open_.append(node)
    #                 for neighbour in graph.edges[node.state]:
    #                     if neighbour not in open_:
    #                         neighbour.append(open_)
    #         while len(open_)>0:
    #             open_.sort()
    #             current_node = open_.pop(0)
    #             if current_node.tag=="NEW":
    #                 for neighbour in graph.edges[current_node.state]:







                    
            
            
            

            

            
