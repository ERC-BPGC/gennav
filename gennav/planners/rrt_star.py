#! /usr/bin/env python
"""
Path planning Code of RRT* with
author: Ojit Mehta(@ojitmehta123)
"""
import numpy as np
import sys
import os


from utils_scan import scan_obstacle_checker, make_obstacles_scan, check_intersection_scan
from utils_scan import adjustable_random_sampler as sampler
from descartes import PolygonPatch
from shapely.geometry import Polygon, Point, LineString
import random
import math, time
import matplotlib.pyplot as plt


def propagate_cost_to_leaves(node_list, parent_node):

        for node in node_list:
            if node.parent == parent_node:
                node.cost = calculate_cost(parent_node, (node.x, node.y))
                propagate_cost_to_leaves(node_list, node)


def calculate_cost(from_node, to_node):
    """ To calculate Cost from from_node --> to_node
        Arguments: 	from_node --> Node from node_list
                    to_node --> tuple
        Return:
                    Value of Cost from start to to_node
    """
    return from_node.cost + math.sqrt((from_node.x - to_node[0])**2 + (from_node.y - to_node[1])**2)



class Node(object):
    """
    Coordinate representation in node form.
    x,y --> Coordinates
    Parent node is the node connected to the present node
    path_x --> X coordinates of the path to this node 
    path_y --> Y coordinates of the path to this node
    Cost --> The cost suffered to get to this node.  
    """

    def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0

    def __str__(self):
        return ("("+str(self.x)+','+str(self.y)+")")




class RRTStar(object):
    """
    RRT star algorithm
    """

    def __init__(self, sample_area,
                    expand_dis=1.0,
                    path_resolution=1.0,
                    goal_sample_rate=0.1,
                    max_iter=500,
                    connect_circle_dist=10.0,
                    search_until_max_iter = True,
                    initial_exploration = 20
                    ):
        """
        start: Start Point. in our case remains(0 , 0) unless specified
        goal: Next goal to be reached
        scan = LaserScan polar distances to Obstacles [r1,r2,r3...] initially assuming every scan occurs at 1 rad interval
        sample_area : Sampling Area (Random Sampler being Used)
        goal_sample_rate = Rate at which goal should be sampled to drive towards goal
        max_iter = Maximum Iterations to be performed
        connect_circle_dist = 10.0, This distance is to be adjusted to get smoother and shortest path.
                              On increasing Path Smoothens, but Processing Increases as well
        search_until_max_iter = If False then gets the path as soon as found
        initial_exploration = Explore initially, Run as RRT for initial_exploration number of Iterations
        """

        self.sample_area = sample_area
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.circle = connect_circle_dist
        self.max_iter = max_iter
        self.search_until_max_iter = search_until_max_iter
        self.initial_explore = initial_exploration

    def __call__(self, goal_point, scan, start_point=[0, 0], animation=False):
        """Plans path from start to goal avoiding obstacles.

        Args:
            start_point: tuple with start point coordinates.
            end_point: tuple with end point coordinates.
            scan: list of obstacles which themselves are list of points
            animation: flag for showing planning visualization (default False)

        Returns:
            A list of points representing the path determined from
            start to goal while avoiding obstacles.
            An list containing just the start point means path could not be planned.
        """

        # Make line obstacles and scan in x,y from scan
        line_obstacles, _ = make_obstacles_scan(scan)

        # Setting Start and End
        self.start = Node(start_point[0], start_point[1])
        self.goal = Node(goal_point[0], goal_point[1])

        # Initialize node with Starting Position
        self.node_list = [self.start]

        # Loop for maximum iterations to get the best possible path
        for iter in range(self.max_iter):

            # Sample a Random point in the sample area
            rnd_point = sampler(self.sample_area, (self.goal.x , self.goal.y), self.goal_sample_rate)
            # Find nearest node to the sampled point
            distance_list = [(node.x - rnd_point[0])**2 + (node.y -
                              rnd_point[1])**2 for node in self.node_list]
            nearest_node = self.node_list[distance_list.index(min(distance_list))]
            # Creating a new Point in the Direction of sampled point
            theta = math.atan2(rnd_point[1] - nearest_node.y,
                               rnd_point[0] - nearest_node.x)
            new_point = nearest_node.x + self.expand_dis*math.cos(theta), \
                        nearest_node.y + self.expand_dis*math.sin(theta)
            
            # Check obstacle collision
            new_point = scan_obstacle_checker(scan, new_point)

            if math.isnan(new_point[0]):
                continue

###############################################################################################################
#THIS WILL ONLY WORK FOR SOME INITIAL ITERATIONS
            #If iterations is less than certain no. try exploring a bit, run similar to RRT    
            if iter<self.initial_explore:

                new_node = Node(new_point[0],new_point[1])
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + math.sqrt((new_node.x-nearest_node.x)**2 + (new_node.y-nearest_node.y)**2)
                
                #Set the path for new node
                present_node = new_node
                px =[] #X-coordinate path
                py=[] #Y-coordinate path

                #Keep on appending path until reaches start
                while present_node.parent != None:
                    px.append(present_node.x)
                    py.append(present_node.y)
                    present_node = present_node.parent
                
                px.append(self.start.x)
                py.append(self.start.y)

                #Setting Path
                new_node.path_x = px[:]
                new_node.path_y = py[:]

                if animation and iter % 5 == 0:
                    self.draw_graph(scan, new_node)
                continue
###############################################################################################################


###############################################################################################################
            #FINDING NEAREST INDICES
            nnode = len(self.node_list) + 1
            #The circle in which to check parent node and rewiring
            r = self.circle * math.sqrt((math.log(nnode) / nnode))
            dist_list = [(node.x - new_point[0])**2 + (node.y - new_point[1])**2 for node in self.node_list]
            #Getting all the indexes within r units of new_node
            nearest_indexes = [dist_list.index(i) for i in dist_list if i <= r ** 2]
###############################################################################################################

###############################################################################################################

            #GETTING THE PARENT NODE FROM NEAREST INDICES FOR BEST PARENT WITH LEAST COST
            costs = []  # List of Total costs from the start to new_node when attached to parent node in node_list

            for index in nearest_indexes:
                near_node = self.node_list[index]
                point_list = [(near_node.x , near_node.y), (new_point[0],new_point[1])]
                if not check_intersection_scan(point_list, line_obstacles):
                    costs.append(near_node.cost + math.sqrt((near_node.x - new_point[0])**2 + (near_node.y - new_point[1])**2))
                else:
                    costs.append(float("inf"))
            
            # If costs is empty continue
            try:
                min_cost = min(costs)
            except:
                continue
            
            # Calculating the minimum cost and selecting the node for which it occurs as parent child
            if min_cost == float("inf"):
                continue

            # Setting the new node as the one with min cost
            min_ind = nearest_indexes[costs.index(min_cost)]
            new_node = Node(new_point[0],new_point[1])
            new_node.parent = self.node_list[min_ind]
            new_node.cost = min_cost
###############################################################################################################


###############################################################################################################
            #REWIRING
            if new_node:
                #First append the node to nodelist
                self.node_list.append(new_node)
                
                #Rewiring
                for ind in nearest_indexes:
                    #Check for Every Nearest Node in node_list the possibility of rewiring to new node
                    node_check = self.node_list[ind]
                    point_list = [(new_node.x , new_node.y), (node_check.x , node_check.y)]

                    #Check if the straight line from new_node to node_check is collision free, all others will automatically be collision free                    
                    no_coll = not check_intersection_scan(point_list, line_obstacles)

                    #Check for Cost improvement
                    cost_improv = new_node.cost + math.sqrt((new_node.x - node_check.x)**2 + (new_node.y - node_check.y)**2) < node_check.cost

                    #If both the above conditions are met, set the parent node of node check to new node
                    if no_coll and cost_improv:
                        node_check.parent = new_node
###############################################################################################################


###############################################################################################################

                #SETTING PATH THE NODE
                present_node = new_node
                px =[]
                py=[]
                while present_node.parent != None:
                    px.append(present_node.x)
                    py.append(present_node.y)
                    present_node = present_node.parent
                px.append(self.start.x)
                py.append(self.start.y)
                new_node.path_x = px[:]
                new_node.path_y = py[:]
###############################################################################################################

            if animation and iter % 5 == 0:
                self.draw_graph(scan, new_node)

###############################################################################################################
            #TO PREEMPT BEFORE REACHING MAX ITERATIONS, ONCE GOAL FOUND
            if (not self.search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node(scan)
                if last_index:
                    path = [[self.goal.x, self.goal.y]]
                    node = self.node_list[last_index]
                    while node.parent is not None:
                        path.append([node.x, node.y])
                        node = node.parent
                    path.append([node.x, node.y])
                    return path
###############################################################################################################

###############################################################################################################

        last_index = self.search_best_goal_node(scan)
        if last_index:
            path = [[self.goal.x, self.goal.y]]
            node = self.node_list[last_index]
            while node.parent is not None:
                path.append([node.x, node.y])
                node = node.parent
            path.append([node.x, node.y])
            return path
        return None
###############################################################################################################
###############################################################################################################
###############################################################################################################

    def draw_graph(self, scan, rnd=None):
        """ For Graphing Purpose """
        plt.clf()
        pt_ang = np.arange(0,2*np.pi,np.pi/180)
        pt_scan = np.array(scan)
        pts = []
        pt_x = np.multiply(pt_scan,np.cos(pt_ang))
        pt_y = np.multiply(pt_scan,np.sin(pt_ang))

        for a,b in zip(pt_x,pt_y):
		    pts.append((a,b))

        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        plt.plot([x for (x, _) in pts], [y for (_, y) in pts],'r.')
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis((-5,5,-5,5))
        plt.grid(True)
        plt.pause(0.01)

    def search_best_goal_node(self,scan):
        dist_to_goal_list = [math.sqrt(
            (n.x - self.goal.x)**2 + (n.y - self.goal.y)**2) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(
            i) for i in dist_to_goal_list if i <= self.expand_dis]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            theta = math.atan2(
                self.node_list[goal_ind].y - self.goal.y, self.node_list[goal_ind].x - self.goal.x)

            t_node = Node(self.node_list[goal_ind].x + math.cos(theta) , self.node_list[goal_ind].y + math.sin(theta))
            if scan_obstacle_checker(scan , (t_node.x , t_node.y)):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None