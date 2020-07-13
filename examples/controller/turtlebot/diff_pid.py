#!/usr/bin/env python

import math
import time
from math import atan2, pi, pow, sqrt

import rospy
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05


class GotoPoint:
    def __init__(self):
        rospy.init_node("turtlebot3_pointop_key", anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = "odom"

        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, "base_footprint", rospy.Time(), rospy.Duration(1.0)
            )
            self.base_frame = "base_footprint"
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(
                    self.odom_frame, "base_link", rospy.Time(), rospy.Duration(1.0)
                )
                self.base_frame = "base_link"
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo(
                    "Cannot find transform between odom and base_link or base_footprint"
                )
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()

        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)

        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        # distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
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
                kp_angle * path_angle + ki_angle * total_angle + kd_distance * ang_error
            )

            move_cmd.angular.z = (control_signal_angle) - rotation
            # move_cmd.linear.x = min(linear_speed * distance, 0.1)
            move_cmd.linear.x = min(control_signal_distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance
            print("Current positin and rotation are: ", (position, rotation))

        (position, rotation) = self.get_odom()
        print("Current positin and rotation are: ", (position, rotation))

        print("reached :)   ^_^")

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()

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

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0)
            )
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

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
state_msg.model_name = "turtlebot3_burger"
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
