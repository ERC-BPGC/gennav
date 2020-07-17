#!/usr/bin/env python

import math
import time
from math import atan2, pow, sqrt

import numpy as np
import rospy
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler

kp_distance = 1
ki_distance = 0
kd_distance = 0

kp_angle = 1
ki_angle = 0
kd_angle = 0


class GotoPoint:
    def __init__(self):
        rospy.init_node("omnibase_differntial", anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_sub)
        self.position = Point()
        self.rotation = 0
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = "world"

        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, "base_link", rospy.Time(), rospy.Duration(1.0)
            )
            self.base_frame = "base_link"
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_link")
            rospy.signal_shutdown("tf Exception")

        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()

        goal_distance = sqrt(
            pow(goal_x - self.position.x, 2) + pow(goal_y - self.position.y, 2)
        )
        # distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0

        while distance > 0.05:
            x_start = self.position.x
            y_start = self.position.y
            # path_angle = error
            path_angle = atan2(goal_y - y_start, goal_x - x_start)
            alpha = path_angle - previous_angle
            ang_error = math.atan2(math.sin(alpha), math.cos(alpha))

            diff_distance = distance - previous_distance

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            control_signal_distance = (
                kp_distance * distance
                + ki_distance * total_distance
                + kd_distance * diff_distance
            )

            control_signal_angle = (
                kp_angle * ang_error + ki_angle * total_angle + kd_distance * ang_error
            )

            move_cmd.angular.z = (control_signal_angle) - self.rotation
            # move_cmd.linear.x = min(linear_speed * distance, 0.1)
            move_cmd.linear.x = min(control_signal_distance, 0.25)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance
            print("Current positin and rotation are: ", (self.position, self.rotation))

        print("Current positin and rotation are: ", (self.position, self.rotation))

        print("reached :)   ^_^")

        # rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        return

    def getkey(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        if x == "s":
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def _odom_sub(self, msg):

        self.position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        orientatioN = efq([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.rotation = orientatioN[2]

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


print(
    "Please enter the lower range for generating x and y coordinates for the starting position of Turtlebot"
)
lower = input()

print(
    "Please enter the upper range for generating x and y coordinates for the starting position of Turtlebot"
)
upper = input()

print("Random initial X and Y coordinates of the Turtlebot are:-")

coord = np.random.uniform(lower, upper, 2)
print("Initial X coordinate: ", coord[0])
print("Initial Y coordinate: ", coord[1])

print(
    "Please enter the lower range for generating angle wrt x axis for the starting position of Turtlebot in degrees"
)
lower_angle = math.radians(input())

print(
    "Please enter the upper range for generating angle wrt x axis for the starting position of Turtlebot in degrees"
)
upper_angle = math.radians(input())

angle = np.random.uniform(lower_angle, upper_angle, 1)
print("Initial starting angle Theta wrt +X axis: ", angle[0])

# initial_position = coord + angle
initial_position = np.concatenate((coord, angle))

# print('(X, Y, Theta):' ,coord[0], coord[1], angle[0])
print("Initial pose is:-")
print("(X, Y, Theta):", initial_position[0], initial_position[1], initial_position[2])

print("Enter final x position")
x_final = input()
print("Enter final y position")
y_final = input()
print("Enter final angle position")
angle_final = input()

final = [x_final, y_final, angle_final]
final_position = np.array(final)

x_input = final_position[0]
y_input = final_position[1]
z_input = final_position[2]


q = quaternion_from_euler(0, 0, initial_position[2])
# state_msg is an object
state_msg = ModelState()
state_msg.model_name = "omnibase"
state_msg.pose.position.x = initial_position[0]
state_msg.pose.position.y = initial_position[1]
state_msg.pose.position.z = 0

state_msg.pose.orientation.x = q[0]
state_msg.pose.orientation.y = q[1]
state_msg.pose.orientation.z = q[2]
state_msg.pose.orientation.w = q[3]

set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
resp = set_state(state_msg)
print(resp)

time.sleep(5)

while not rospy.is_shutdown():
    GotoPoint()
