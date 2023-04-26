#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, sin, cos
from tf.transformations import euler_from_quaternion
import numpy as np
from image_processor.msg import object_info
from image_processor.msg import all_objects_info

#import sys

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080

# class husky():
#     def __init__(self):
#         # Creating our node,publisher and subscriber
#         rospy.init_node('movement_processor', anonymous=True)
#         self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
#         self.pose_subscriber = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.callback)
#         self.pose = Odometry()
#         self.rate = rospy.Rate(10)
#
#         # Callback function implementing the pose value received
#
#     def callback(self, data):
#         self.pose = data.pose.pose.position
#         self.orient = data.pose.pose.orientation
#         self.pose.x = round(self.pose.x, 4)
#         self.pose.y = round(self.pose.y, 4)
#
#     def move2goal(self):
#         K1 = 0.5
#         K2 = 0.5
#         goal_pose_ = Odometry()
#         goal_pose = goal_pose_.pose.pose.position
#         goal_pose.x = input("Set your x goal:")
#         goal_pose.y = input("Set your y goal:")
#         distance_tolerance = input("Set your tolerance:")
#         vel_msg = Twist()
#         r = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
#         while r >= distance_tolerance:
#
#             r = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
#             psi = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
#             orientation_list = [self.orient.x, self.orient.y, self.orient.z, self.orient.w]
#             (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
#             theta = yaw
#             phi = theta - psi
#             if phi > np.pi:
#                 phi = phi - 2 * np.pi
#             if phi < -np.pi:
#                 phi = phi + 2 * np.pi
#
#             vel_msg.linear.x = K1 * r * cos(phi)
#             vel_msg.angular.z = -K1 * sin(phi) * cos(phi) - (K2 * phi)
#
#             # Publishing input
#             self.velocity_publisher.publish(vel_msg)
#             self.rate.sleep()
#         # Stopping our robot after the movement is over
#         vel_msg.linear.x = 0
#         vel_msg.angular.z = 0
#         self.velocity_publisher.publish(vel_msg)


# EVERYTHING BEFORE THIS IS TAKEN FROM THE FOLLOWING URL:
# https://github.com/engcang/husky/blob/master/Python-Kinematic-Position-Control/README.md

# Husky class
class Husky:

    # Initialize husky object
    def __init__(self):
        self.move_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)

    # Callback function for moving robot
    def callback(self, data):

        # Bool values to record whether person, flag, or enemy is in frame
        person_bool = False
        flag_bool = False
        enemy_bool = False
        move_cmd = Twist()
        spin_cmd = Twist()
        MIN_FOR_MOVE_FORWARD, MAX_FOR_MOVE_FORWARD = 0  # DELETE THIS LATER

        for i in data.objects:
            if i.Class == "person":
                person_bool = True
            elif i.Class == "flag":
                flag_bool = True
            elif i.Class == "enemy":
                enemy_bool = True   # To be used in the future

        if person_bool or not flag_bool:
            spin_cmd.angular.z = 2
            self.move_pub.Publish(spin_cmd)
            rospy.Rate(0.5).sleep()
        else:
            flag_obj = next(obj for obj in data.objects if obj.Class == "flag")
            if MIN_FOR_MOVE_FORWARD < flag_obj.xcenter < MAX_FOR_MOVE_FORWARD: # Minimum and maximum x center for the robot to proceed forward; test this out
                move_cmd.linear.x = 1
                self.move_pub.Publish(move_cmd)
                rospy.Rate(10).sleep()
            else:
                spin_cmd.angular.z = (IMAGE_WIDTH/2 - flag_obj.xcenter)/IMAGE_WIDTH
                self.move_pub.Publish(spin_cmd)
                rospy.Rate(1).sleep()
            # move_cmd.linear.x

        # data.INSERT_FIELD_HERE <- example of what to do

    # Initialize ROS elements function
    def ros_init(self):
        rospy.init_node('movement_processor')
        rospy.Subscriber('/image_processor/all_objects', all_objects_info, self.callback)
        rospy.spin()

# if __name__ == '__main__':
#     x = husky()
#     while True:
#         try:
#             x.move2goal()
#         except rospy.ROSInterruptException:
#             pass


if __name__ == '__main__':

    # Initialize husky node and get ROS working
    husky_node = Husky()
    husky_node.ros_init()
