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
        self.velocityPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("/odom", Odometry, self.poseCallback)
        movelength = 0.3
        maximum_speed = 0.025
        self.move_v1(maximum_speed, movelength, True)
        rospy.loginfo("All step completed. Trying auto termination")
        raise rospy.ROSInterruptException

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
        #setting intial speed
        VelocityMessage = Twist()
        initial_turtlebot_odom_pose = Odometry()
        if (isForward):
                VelocityMessage.linear.x =abs(speed)
        else:
                VelocityMessage.linear.x =-abs(speed)
        VelocityMessage.linear.y =0
        VelocityMessage.linear.z =0
        VelocityMessage.angular.x = 0
        VelocityMessage.angular.y = 0
        VelocityMessage.angular.z = 0

        #initial_turtlebot_odom_pose = Odometry()
        #TODO CHECK IMU!
        distance_moved = 0.0
        loop_rate = rospy.Rate(20)
        waiting_rate = rospy.Rate(1)
        waiting_rate.sleep() #waiting for initial odom pos
        initial_turtlebot_odom_pose = copy.deepcopy(self.turtlebot_odom_pose)
        rospy.loginfo('Initial x={0}'.format(initial_turtlebot_odom_pose.pose.pose.position.x))
        while True :
                rospy.loginfo('distance_moved={0}'.format(distance_moved))
                self.velocityPublisher.publish(VelocityMessage)
                distance_moved = abs(self.turtlebot_odom_pose.pose.pose.position.x - initial_turtlebot_odom_pose.pose.pose.position.x)  
                if not (distance_moved<distance):
                    break
                loop_rate.sleep()
        VelocityMessage.linear.x =0
        self.velocityPublisher.publish(VelocityMessage)
        rospy.loginfo('Final x={0}'.format(self.turtlebot_odom_pose.pose.pose.position.x))


if __name__ == '__main__':
    try:
        free_space_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
