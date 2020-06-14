#!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import Pose

import numpy as np
# from collections import deque

# from .samplers import uniform_adjustable_random_sampler as sampler

# import time

class random_sample_area():
    def __init__(self, x_min, x_max, y_min, y_max, z_min = 0, z_max = 0):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max


class Node():
    def __init__(self, position):
        self.position = position
        self.cost = 0
        self.parent = None

    def __str__(self):
        return "("+str(self.position[0])+','+str(self.position[1])+','+str(self.position[2])+")"


class RRTSTAR():
    def __init__(self, collision_check_function, sample_space, max_step_size, max_iterations, goal_sample_rate = 0.2):
        
        self.collision_check_function = collision_check_function
        self.sample_space = sample_space
        self.max_iterations = max_iterations
        self.max_step_size = max_step_size
        self.goal_sample_rate = goal_sample_rate


    def generate_random_node(self):
        """
        Generates random node in the given sampling area for RRTstar 
        """
        if np.random.random_sample() > self.goal_sample_rate:  
            x = np.random.uniform(self.sample_space.x_min, self.sample_space.x_max)
            y = np.random.uniform(self.sample_space.y_min, self.sample_space.y_max)
            z = np.random.uniform(self.sample_space.z_min, self.sample_space.z_max)
            node = Node(np.array((x,y,z)))
        else:
            node = self.goal_node

        return node


    def euclidian_norm(self, n1, n2):
        """
        Calculates euclidian distance between n1 and n2
        """
        return np.linalg.norm(n1.position - n2.position)
    

    def get_nearest_node(self, n):
        """
        Finds the nearest node to n
        """
        distances = [self.euclidian_norm(n, node) for node in self.nodes]
        # print(distances)
        near_ind = distances.index(min(distances))
        return self.nodes[near_ind]


    def find_nearest_nodes(self, n, const=5):
        '''
        Finds nodes in radius r (formula from sertac karaman's paper)
        We have to tweak the const value
        '''
        if self.sample_space.z_min == 0 and self.sample_space.z_max == 0:
            radius = const * np.sqrt(np.log(len(self.nodes)+1)/ len(self.nodes))
        else:
            radius = const * np.cbrt(np.log(len(self.nodes)+1)/ len(self.nodes))

        # print "radius ", radius

        distances = [self.euclidian_norm(n, node) for node in self.nodes]

        near_ind = [distances.index(i) for i in distances if i < radius]
        # print("near ind",near_ind)
        nodes = [self.nodes[i] for i in near_ind]
        
        return nodes


    def steer(self, n1, n2):
        """
        Creates a new node in the direction of n2 from n1
        """
        v = n2.position - n1.position
        v_mod = np.linalg.norm(v)

        if v_mod == 0:
            return n1
        if v_mod < self.max_step_size:
            # print "returning n2"
            return n2
        
        return Node(n1.position + v * self.max_step_size / v_mod)


    def obstacle_free(self, n1, n2):
        """
        Returns true if the path from n1 to n2 is collision free
        """
        v = n2.position - n1.position
        v_mod = np.linalg.norm(v)

        for i in range(int(v_mod/self.max_step_size)):
            temp_node = Node(n1.position + i * self.max_step_size * v / v_mod)
            
            if self.collision_check_function(temp_node):
                return False    
        
        return True


    def select_parent(self, new_node, nodes):
        """
        New node selects the parent with the lowest cost
        """
        if len(nodes) == 0:
            return new_node
        
        costs = []
        for node in nodes:
            if self.obstacle_free(new_node, node):
                costs.append(node.cost + self.euclidian_norm(new_node, node))
            else:
                costs.append(np.Inf)
        
        if min(costs) == np.Inf:
            return new_node

        min_cost_node = nodes[costs.index(min(costs))]
        new_node.cost = min(costs)
        new_node.parent = min_cost_node
        return new_node
        

    def rewire(self, n, nodes):
        """
        Rewires the nearest_nodes such that the nodes have min cost
        """
        for node in nodes:
            distance = self.euclidian_norm(n, node)
            temp_cost = distance + n.cost
            if node.cost > temp_cost and self.obstacle_free(n, node):
                node.parent = n
                node.cost = temp_cost


    def best_last_node(self):
        """
        Gives the best last node
        """
        # rospy.loginfo("IN best_last_node func")
        if self.goal_node in self.nodes:
            return self.goal_node 
        
        distances_to_goal = [self.euclidian_norm(self.goal_node, node) for node in self.nodes]
        # print distances_to_goal
        goal_indices = [distances_to_goal.index(distance) for distance in distances_to_goal if distance <= self.max_step_size]
        if len(goal_indices) == 0:
            self.goal_node.parent = self.get_nearest_node(self.goal_node)            
            return self.goal_node
            
        min_cost = min([self.nodes[i].cost for i in goal_indices])
        for i in goal_indices:
            if self.nodes[i].cost == min_cost:
                return self.nodes[i]  
        # return None


    def get_path(self, n):
        """
        Gives the path from last node
        """
        path = []
        while n is not None:
            path.append(n)
            n = n.parent

        if self.start_node not in path:
            path.append(self.start_node)
        
        path.reverse()
        return path


    def plan(self, start_pose, goal_pose):
        """
        Plans the path
        """
        self.start_node = start_node
        self.goal_node = goal_node

        self.nodes = [self.start_node]
        
        ### Expanding the tree (make a function for expansion?)
        for i in range(self.max_iterations):
            random_node = self.generate_random_node()
            nearest_node = self.get_nearest_node(random_node)
            new_node = self.steer(nearest_node, random_node)
            
            print i

            if self.obstacle_free(nearest_node,new_node):
                nearest_nodes = self.find_nearest_nodes(new_node) # will have to choose the constant properly
                new_node = self.select_parent(new_node, nearest_nodes)
                self.nodes.append(new_node)
                self.rewire(new_node, nearest_nodes)
        
        ### Expansion done

        last_node = self.best_last_node()
        if last_node == None: ## returns empty path
            return None
            
        path = self.get_path(last_node)

        # for i in range(len(path)-1):
        #     if not self.obstacle_free(path[i],path[i+1]):
        #         print "obstacle in bet"
        #         self.plan(self.node_to_pose(self.start_node), self.node_to_pose(self.goal_node))
        
        print("Path is:")
        for node in path:
            print(str(node))

        print "no of nodes", len(self.nodes)

        # print poses
        return path
