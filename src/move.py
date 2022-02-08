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
        movelength = 0.1
        maximum_speed = 0.01
        # self.move_v1(maximum_speed, movelength, True)
        self.rotate(0.2, self.degree2radian(90.0),True)
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

    def rotate(self,angular_velocity,radians,clockwise):
        rotateMessage = Twist()
        listener = tf.TransformListener()
        #init_transform: is the transformation before starting the motion
        init_transform = geometry_msgs.msg.TransformStamped()
        #current_transformation: is the transformation while the robot is moving
        current_transform = geometry_msgs.msg.TransformStamped()
        
        angle_turned = 0.0

        angular_velocity = (-angular_velocity, ANGULAR_VELOCITY_MINIMUM_THRESHOLD)[angular_velocity > ANGULAR_VELOCITY_MINIMUM_THRESHOLD]

        while(radians < 0):
            radians += 2* pi

        while(radians > 2* pi):
            radians -= 2* pi
        
        listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0) )
        (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
        #listener.lookupTransform("/base_footprint", "/odom", rospy.Time(0),init_transform)
        
        init_transform.transform.translation = trans
        init_transform.transform.rotation =rot

        #since the rotation is only in the Z-axes 
        #start_angle = tf.transformations.#0.5 * sqrt(rot[2] ** 2)
        euler = tf.transformations.euler_from_quaternion(rot)
        roll = euler[0]
        pitch = euler[1]
        start_angle = euler[2]

        rotateMessage.linear.x = rotateMessage.linear.y = 0.0
        rotateMessage.angular.z = angular_velocity

        if (clockwise):
            rotateMessage.angular.z = -rotateMessage.angular.z
        
        
        loop_rate = rospy.Rate(20)
        
        while True:
            rospy.loginfo("Turtlebot is Rotating")

            self.velocityPublisher.publish(rotateMessage)
         
            loop_rate.sleep()
                    
            #rospy.Duration(1.0)

            try:

                #wait for the transform to be found
                listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0) )
                #Once the transform is found,get the initial_transform transformation.
                #listener.lookupTransform("/base_footprint", "/odom",rospy.Time(0))
                (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.Duration(1.0)

            current_transform.transform.translation = trans
            current_transform.transform.rotation =rot

            #since the rotation is only in the Z-axes 
            #end_angle = 0.5 * sqrt( rot[2] ** 2)
            euler = tf.transformations.euler_from_quaternion(rot)
            roll = euler[0]
            pitch = euler[1]
            end_angle = euler[2]
            
            angle_turned = abs(end_angle - start_angle)
            print("angle_turned: %s" %angle_turned)
            if (angle_turned > radians):
                break
        self.velocityPublisher.publish(rotateMessage)
        loop_rate.sleep()
    
    def degree2radian(self, degreeAngle):
        return (degreeAngle/57.2957795)

if __name__ == '__main__':
    try:
        free_space_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
