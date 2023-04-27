#!/usr/bin/env python3

# READING FROM DEPTH IMAGE STILL NEEDS TO BE INTEGRATED
# FOR OBSTACLE AVOIDANCE

import rospy
from geometry_msgs.msg import Twist
from image_processor.msg import object_info
from image_processor.msg import all_objects_info

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
MIN_MOVE = IMAGE_WIDTH/2 - 150
MAX_MOVE = IMAGE_WIDTH/2 + 150


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
        enemy_bool = False  # To be used in the future
        move_cmd = Twist()
        spin_cmd = Twist()

        # Setting bool values based on what is in frame
        for i in data.objects:
            if i.Class == "person":
                person_bool = True
            elif i.Class == "flag":
                flag_bool = True
            elif i.Class == "enemy":
                enemy_bool = True   # To be used in the future

        # Robot turns if person is in way and/or flag is not present
        if person_bool or not flag_bool:
            spin_cmd.angular.z = 2
            self.move_pub.publish(spin_cmd)
            rospy.Rate(0.5).sleep()

        # Head toward flag if in view
        else:
            flag_obj = next(obj for obj in data.objects if obj.Class == "flag")
            # Minimum and maximum x center for the robot to proceed forward; test this out
            # with several different values to see what works best
            if MIN_MOVE < flag_obj.xcenter < MAX_MOVE:
                move_cmd.linear.x = 1
                self.move_pub.publish(move_cmd)
                rospy.Rate(10).sleep()
            else:
                spin_cmd.angular.z = (IMAGE_WIDTH/2 - flag_obj.xcenter)/IMAGE_WIDTH
                self.move_pub.publish(spin_cmd)
                rospy.Rate(1).sleep()

    # Initialize ROS elements function
    def ros_init(self):
        rospy.init_node('movement_processor')
        rospy.Subscriber('/image_processor/all_objects', all_objects_info, self.callback)
        rospy.spin()


if __name__ == '__main__':

    # Initialize husky node and get ROS working
    husky_node = Husky()
    husky_node.ros_init()
