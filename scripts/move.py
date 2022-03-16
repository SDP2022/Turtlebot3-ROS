#!/usr/bin/env python

import math
import rospy
import tf
import numpy
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Point, Quaternion
from math import *
from math import pi
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
import copy
import PyKDL


class free_space_navigation():
    def __init__(self):
        # Node init
        rospy.init_node('free_space_navigation', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Publisher to control node()
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # move_cmd = Twist()
        # move_cmd.linear.x = 0.25
        # move_cmd.linear.y = 0
        # move_cmd.linear.z = 0
        # move_cmd.angular.x = 0
        # move_cmd.angular.y = 0
        # move_cmd.angular.z = 0
        # self.cmd_vel.publish(move_cmd)
        # r = rospy.Rate(1)
        # r.sleep()
        # self.cmd_vel.publish(Twist())

        # Initial velocity parameters
        self.linear_vel = 0.25
        self.angular_vel = 0.1

        # Initial transform
        self.initial_transform()

        # Start moving
        self.move(1)

        rospy.loginfo("All step completed. Trying auto termination")
        raise rospy.ROSInterruptException
    
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stopping move node")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def initial_transform(self):
        self.tf_listener = tf.TransformListener()
        r = rospy.Rate(20)
        r.sleep()
        self.odom_frame = '/odom'
        # Look if robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")
        rospy.loginfo('transform complete, base_frame={0}'.format(self.base_frame))

    def move(self, goal_distance):
        move_cmd = Twist()
        move_cmd.linear.x = 0.25
        move_cmd.linear.y = 0
        move_cmd.linear.z = 0
        move_cmd.angular.x = 0
        move_cmd.angular.y = 0
        move_cmd.angular.z = 0

        r = rospy.Rate(20)
        r.sleep()

        position = Point()
        (position, rotation) = self.get_odom()
        x_start = position.x
        y_start = position.y
        # Keep track of the distance traveled
        current_distance = 0
        # Enter the loop to move along a side
        while current_distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)

            r.sleep()

            # Get the current position
            (position, rotation) = self.get_odom()

            # Compute the Euclidean distance from the start
            current_distance = sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))
            rospy.loginfo("current_distance={0}".format(current_distance))
        rospy.loginfo("Move distance={0} completed. Acutal distance={1}".format(goal_distance, current_distance))

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    #Ref: https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_nav/src/transform_utils.py
    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

if __name__ == '__main__':
    try:
        free_space_navigation()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
