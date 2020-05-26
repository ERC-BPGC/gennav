#!/usr/bin/env python
#Add Imports here
import os
try:

    from geometry_msgs.msg import Point32, PoseArray
    import sensor_msgs
    import sys
    import gennav

except Exception as e:
    print ("Import Error in: " + os.path.abspath(__file__)) 
    print (e)

class Planner (object):

    """ This Class is responsible for Initial Path to be planned.
        
        It uses the following api:
            plan

        and sets the following attributes:
            state_space_range
            observation_range  
    """
    
    state_space_range = ( Point32(-float('inf') , -float('inf'), -float('inf')) , Point32(-float('inf') , -float('inf'), -float('inf')) )
    observation_range = None # To be set depending on the max range of the sensors
     
    def plan (self, start, end, obstacle):
        """ 
      
            This method will be responsible for planning
            the path from start to end.
            
            Properties of the planner used is free to be
            chosen and should be handled in the subclass

                Args: 
                    start (geometry_msgs/Point32) : start point of the planning
                    end (geometry_msgs/Point32)   : end point or goal point.
                    obstacle (object)             : obstacles as taken by the planner & checker.

                Returns:
                    path (geometry_msgs/PoseArray): WayPoints  
      
        """
        raise NotImplementedError
    
    def check (self ,path ,obstacle):
        """ 
            Checks if the path has any obstacles
            
            Args: 
                path (geometry_msgs/PoseArray) : Path to be checked
                obstacle (object)              : obstacles as take by the planner.
                
            Returns:
                obst_free (bool)               : True if path is obstacle free else false

        """
        raise NotImplementedError


class ObstacleChecker (object):
    """
        This Class is responsible for dealing with obstacles in the environment
        
        API:
            point_collide
            path_collide
            update_data

        pt_cloud (sensor_msgs/LaserScan) or (geometry_msgs/PointCloud) : Environment Perception
    """

    scan = None
    bot_size = None
    ang_range = None

    def point_collide (self, point):
        """ 
            Check wether the sample point lies in the obstacle
            
            Args:
                point (geomemtry_msgs/Point32)   : Point to be checked
                is_scan (bool)                   : If True then considered as scan else considered as PointCloud
            Returns:
                pt_obst_free (bool)              : True if point lies in obstacle else false
        """
        raise NotImplementedError
        

    def path_collide (self, path):
        """ 
            Check wether the path is being intersected by any obstacle.
            
            Args:
                path (geomemtry_msgs/PoseArray)   : Point to be checked
            
            Returns:
                obst_free (bool)                  : True if path lies is obstacle else false
        """
        raise NotImplementedError


class Controller (object):
    """ 
        This is responsible  for controlling the velocity of the robot
        Use and setting of cocntrol algo in subclass
            
    """

    def move_bot (self, present_pose, next_pose):
        """ 
            Move the Bot to next_pose from present_pose
            
            Args:
                present_pose (geometry_msgs/PoseStamped)          :  current position
                next_pose (geometry_msgs/PoseStamped)             :  Waypoint Position

            Return:
                vel (geometry_msgs/TwistStamped)                  :  Command Velocity

        """
        raise NotImplementedError