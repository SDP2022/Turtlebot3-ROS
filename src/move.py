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
        rospy.loginfo("Stopping move node")
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
        VelocityMessage = Twist()
        listener = tf.TransformListener()
        initial_turtlebot_odom_pose = Odometry()
        init_transform = geometry_msgs.msg.TransformStamped()
        current_transform = geometry_msgs.msg.TransformStamped()
        if (isForward):
                VelocityMessage.linear.x =abs(speed)
        else:
                VelocityMessage.linear.x =-abs(speed)
        VelocityMessage.linear.y =0
        VelocityMessage.linear.z =0
        VelocityMessage.angular.x = 0
        VelocityMessage.angular.y = 0
        VelocityMessage.angular.z = 0
        distance_moved = 0.0
        loop_rate = rospy.Rate(20)
        initial_turtlebot_odom_pose = copy.deepcopy(self.turtlebot_odom_pose)
        while True :
                rospy.loginfo("Turtlebot moves forwards")
                self.velocityPublisher.publish(VelocityMessage)
                loop_rate.sleep()
                distance_moved = distance_moved+abs(0.5 * sqrt(((self.turtlebot_odom_pose.pose.pose.position.x-initial_turtlebot_odom_pose.pose.pose.position.x) ** 2) +
                    ((self.turtlebot_odom_pose.pose.pose.position.x-initial_turtlebot_odom_pose.pose.pose.position.x) ** 2)))    
                if not (distance_moved<distance):
                    break
        VelocityMessage.linear.x =0
        self.velocityPublisher.publish(VelocityMessage)


if __name__ == '__main__':
    try:
        free_space_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
