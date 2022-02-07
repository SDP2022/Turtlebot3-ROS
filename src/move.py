#!/usr/bin/env python

import rospy
import tf
import numpy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
import copy


class free_space_navigation():
    def __init__(self):
        rospy.init_node('move', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.turtlebot_odom_pose = Odometry()
        pose_message = Odometry()
        self.velocityPublisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.poseCallback)
        movelength = 1
        self.move_v1(0.1, movelength, True)
        rospy.loginfo("All step completed.. Use control+C to terminated")

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stopping moving forward")
        self.velocityPublisher.publish(Twist())
        rospy.sleep(1)

    def poseCallback(self, pose_message):
        self.turtlebot_odom_pose.pose.pose.position.x = pose_message.pose.pose.position.x
        self.turtlebot_odom_pose.pose.pose.position.y = pose_message.pose.pose.position.y
        self.turtlebot_odom_pose.pose.pose.position.z = pose_message.pose.pose.position.z
        self.turtlebot_odom_pose.pose.pose.orientation.w = pose_message.pose.pose.orientation.w
        self.turtlebot_odom_pose.pose.pose.orientation.x = pose_message.pose.pose.orientation.x
        self.turtlebot_odom_pose.pose.pose.orientation.y = pose_message.pose.pose.orientation.y
        self.turtlebot_odom_pose.pose.pose.orientation.z = pose_message.pose.pose.orientation.z

    def move_v1(self, speed, distance, isForward):
        # setup initial moving message
        VelocityMessage = Twist()
        listener = tf.TransformListener()
        if (isForward):
            VelocityMessage.linear.x = abs(speed)
        else:
            VelocityMessage.linear.x = -abs(speed)
        VelocityMessage.linear.y = 0
        VelocityMessage.linear.z = 0
        VelocityMessage.angular.x = 0
        VelocityMessage.angular.y = 0
        VelocityMessage.angular.z = 0
        # wait for transform
        listener = tf.TransformListener()
        try:
            listener.waitForTransform(
                "/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0))
            (trans, rot) = listener.lookupTransform(
                '/base_footprint', '/odom', rospy.Time(0))
            start = 0.5 * sqrt(trans[0] ** 2 + trans[1] ** 2)
        except Exception:
            rospy.Duration(1.0)
        # moving forward (panza vor!)
        distance_moved = 0
        loop_rate = rospy.Rate(10)
        while True:
            rospy.loginfo("Turtlebot moves forwards")
            self.velocityPublisher.publish(VelocityMessage)
            loop_rate.sleep()
            try:
                listener.waitForTransform(
                    "/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0))
                (trans, rot) = listener.lookupTransform(
                    '/base_footprint', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.Duration(1.0)
            end = 0.5 * sqrt(trans[0] ** 2 + trans[1] ** 2)
            distance_moved = distance_moved + \
                abs(abs(float(end)) - abs(float(start)))
            if not (distance_moved < distance):
                break
        VelocityMessage.linear.x = 0
        self.velocityPublisher.publish(VelocityMessage)
        rospy.loginfo("Move {0} completed. Turtlebot stopped".format(distance))


if __name__ == '__main__':
    try:
        free_space_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
