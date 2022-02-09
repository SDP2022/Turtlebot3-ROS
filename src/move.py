#!/usr/bin/env python

import math
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
        self.velocityPublisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.poseCallback)
        for _ in range(0, 4):
            self.move_v1(0.025, 0.5, True)
            self.rotate_v1(0.1, 90.0, True)
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
        # setting intial speed
        VelocityMessage = Twist()
        initial_turtlebot_odom_pose = Odometry()
        if (isForward):
            VelocityMessage.linear.x = abs(speed)
        else:
            VelocityMessage.linear.x = -abs(speed)
        VelocityMessage.linear.y = 0
        VelocityMessage.linear.z = 0
        VelocityMessage.angular.x = 0
        VelocityMessage.angular.y = 0
        VelocityMessage.angular.z = 0

        #initial_turtlebot_odom_pose = Odometry()
        # TODO CHECK IMU!
        distance_moved = 0.0
        loop_rate = rospy.Rate(20)
        waiting_rate = rospy.Rate(1)
        waiting_rate.sleep()  # waiting for initial odom pos
        initial_turtlebot_odom_pose = copy.deepcopy(self.turtlebot_odom_pose)
        rospy.loginfo('Initial x={0}'.format(
            initial_turtlebot_odom_pose.pose.pose.position.x))
        while True:
            rospy.loginfo('distance_moved={0}'.format(distance_moved))
            self.velocityPublisher.publish(VelocityMessage)
            distance_moved = sqrt(((self.turtlebot_odom_pose.pose.pose.position.x-initial_turtlebot_odom_pose.pose.pose.position.x) ** 2) +
                                  ((self.turtlebot_odom_pose.pose.pose.position.y-initial_turtlebot_odom_pose.pose.pose.position.y) ** 2))
            if not (distance_moved < distance):
                break
            loop_rate.sleep()
        VelocityMessage.linear.x = 0
        self.velocityPublisher.publish(VelocityMessage)
        rospy.loginfo('Final x={0}'.format(
            self.turtlebot_odom_pose.pose.pose.position.x))

    def rotate_v1(self, angular_velocity, degree, isClockwise):
        # radians only allows positive. and limit to 0 and 360
        # to target support up to 360
        # since we using to target.we needs limit angular_vel to 0.2 or 0.1
        # setting intial speed
        radians = self.degree2radian(degree)
        rospy.loginfo(radians)
        VelocityMessage = Twist()
        initial_turtlebot_odom_pose = Odometry()
        VelocityMessage.linear.x = 0
        VelocityMessage.linear.y = 0
        VelocityMessage.linear.z = 0
        VelocityMessage.angular.x = 0
        VelocityMessage.angular.y = 0
        if (isClockwise):
            VelocityMessage.angular.z = abs(angular_velocity)
        else:
            VelocityMessage.angular.z = -abs(angular_velocity)

        loop_rate = rospy.Rate(20)
        waiting_rate = rospy.Rate(1)
        waiting_rate.sleep()  # waiting for initial odom pos
        initial_turtlebot_odom_pose = copy.deepcopy(self.turtlebot_odom_pose)
        initial_yaw = self.calculate_yaw_from_quaternion(
            initial_turtlebot_odom_pose.pose.pose.orientation, isClockwise)
        if isClockwise:
            target_yaw = initial_yaw + radians
        else:
            target_yaw = initial_yaw - radians
        rospy.loginfo(target_yaw)
        if target_yaw >= 2 * pi:
            target_yaw -= 2 * pi
        elif target_yaw <= -2 * pi:
            target_yaw += 2 * pi
        rospy.loginfo(target_yaw)
        while True:
            self.velocityPublisher.publish(VelocityMessage)
            orientation = self.turtlebot_odom_pose.pose.pose.orientation
            current_yaw = self.calculate_yaw_from_quaternion(
                orientation, isClockwise)
            rospy.loginfo('yaw={0} target={1}'.format(current_yaw, target_yaw))
            if abs(target_yaw - current_yaw) <= 0.02:
                break
            loop_rate.sleep()
        VelocityMessage.angular.z = 0
        self.velocityPublisher.publish(VelocityMessage)
        rospy.loginfo('Final yaw={0}. Err={1}'.format(
            current_yaw, abs(target_yaw - current_yaw)))

    def calculate_yaw_from_quaternion(self, orientation, isClockwise):
        siny_cosp = 2 * (orientation.w * orientation.z +
                         orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y +
                             orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        if isClockwise and yaw <= 0:
            yaw += 2 * pi
        elif not isClockwise and yaw >= 0:
            yaw -= 2 * pi
        return yaw

    def degree2radian(self, degreeAngle):
        return (degreeAngle/57.2957795)


if __name__ == '__main__':
    try:
        free_space_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
