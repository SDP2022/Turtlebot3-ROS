#!/usr/bin/env python

from painted.srv import *
import math
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist, Point, Quaternion
from math import *
from math import pi, radians
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from painted.msg import State
import tf2_ros
import copy
import PyKDL
import json

NAME = 'control_node'


class control_node():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Publisher to control node()
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.control_node_service = rospy.Service(
            'control_service', ControlCommand, self.control_command_callback)
        
        # Publisher of state
        self.state_pub = rospy.Publisher('/state', State, queue_size=10)
        
        # Subscriber to listen for pause request
        self.state_sub = rospy.Subscriber('/state', State, self.pause_callback)
        self.pause_requested = False
        
        # Subscriber for object detection
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.ls_callback)
        
        #Some parameters for easy tuning of object avoidance
        self.ranges = None
        self.distance = 0.1

        # Initial velocity parameters
        self.linear_vel = 0.1
        self.angular_vel = 0.1
        self.angular_tolerance = radians(2.5)

        # Initial transform
        rospy.sleep(10.0)
        self.initial_transform()

        # Web message
        self.pub_web = rospy.Publisher('web_messages', String, queue_size=10)

        self.log_info("Starting %s service" % (NAME))

    def shutdown(self):
        # stop turtlebot
        self.log_info("Stopping %s node" % (NAME))
        self.cmd_vel.publish(Twist())
        self.log_info("Stop message publish success")

    def initial_transform(self):
        self.tf_listener = tf.TransformListener()
        r = rospy.Rate(20)
        r.sleep()
        self.odom_frame = '/odom'
        # Look if robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(
                    self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                self.log_info(
                    "Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")
        self.log_info(
            'transform complete, base_frame={0}'.format(self.base_frame))
        
    def ls_callback(self, msg):
        self.ranges = msg.ranges
        
    def pause_callback(self, msg):
        m = msg.as_state 
        if m== State().AS_REQUEST_PAUSE:
            self.pause_requested = True
            

    def control_command_callback(self, req):
        print("Executing displacement=%s rotation=%s" %
              (req.displacement, req.rotation))
        if req.displacement != 0:
            return ControlCommandResponse(self.move(req.displacement))
        if req.rotation != 0:
            return ControlCommandResponse(self.rotation(radians(req.rotation)))
        return ControlCommandResponse(False)

    def move(self, goal_distance):
        if goal_distance < 0:
            goal_distance = -goal_distance
            move_cmd = Twist()
            move_cmd.linear.x = -self.linear_vel
        else:
            move_cmd = Twist()
            move_cmd.linear.x = self.linear_vel
        object_detect = False
        r = rospy.Rate(20)
        r.sleep()

        position = Point()
        (position, rotation) = self.get_odom()
        x_start = position.x
        y_start = position.y
        print("x_start={0} y_start={1}".format(x_start, y_start))
        # Keep track of the distance traveled
        current_distance = 0
        # Enter the loop to move along a side
        while current_distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)
            
            if self.ranges[0] < self.distance or self.pause_requested:
                move_cmd = Twist()
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0
                object_detect = True
                # self.pause_requested = False
                break

            r.sleep()

            # Get the current position
            (position, rotation) = self.get_odom()

            # Compute the Euclidean distance from the start
            current_distance = sqrt(pow((position.x - x_start), 2) +
                                    pow((position.y - y_start), 2))
            # self.log_info("current_distance={0} position.x={1} position.y={2}".format(
            #     current_distance, position.x, position.y))
        self.cmd_vel.publish(Twist())
        if object_detect:
            self.state_pub.publish(State(3))
            self.log_info("Object detected: robot stopping")
            self.pub_web.publish(self.make_web_message(
                    "warning", "Object detected: robot stopping"))
        elif self.pause_requested:
            self.state_pub.publish(State(3))
            self.pause_requested = False
            self.log_info("Paused; robot stopping")
            self.pub_web.publish(self.make_web_message(
                    "warning", "Paused; robot stopping"))
        else:
            self.state_pub.publish(State(2))
        self.log_info("Move distance={0} completed. Acutal distance={1}".format(
            goal_distance, current_distance))
        if object_detect:
            return False
        else:
            return True

    def rotation(self, goal_angle):
        move_cmd = Twist()
        if goal_angle > 0:
            move_cmd.angular.z = -self.angular_vel
        else:
            move_cmd.angular.z = self.angular_vel

        r = rospy.Rate(20)
        r.sleep()

        position = Point()
        (position, rotation) = self.get_odom()
        x_start = position.x
        y_start = position.y
        # Keep track of the distance traveled
        last_angle = rotation
        current_angle = 0

        while abs(current_angle + self.angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)

            r.sleep()

            # Get the current position
            (position, rotation) = self.get_odom()

            # Compute the Euclidean distance from the start
            delta_angle = self.normalize_angle(rotation - last_angle)
            current_angle += delta_angle
            last_angle = rotation

            self.log_info("current_rotation={0}".format(current_angle))
        self.cmd_vel.publish(Twist())
        self.log_info("Rotation={0} completed. Acutal rotation={1}".format(
            goal_angle, current_angle))
        return True

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            self.log_info("TF Exception")
            return
        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    # Ref: https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_nav/src/transform_utils.py
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

    def make_web_message(self, alert_type, message):
        return json.dumps({
            "alert_type": alert_type,
            "message": message
        })

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    control_node()
    rospy.spin()
    # try:
    #     control_node()
    #     # spin() keeps Python from exiting until node is shutdown
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("node terminated.")
