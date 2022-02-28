#!/usr/bin/env python

from painted.srv import *
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

NAME = 'control_node'

def control_command_callback(req):
    print("Executing displacement=%s rotation=%s" % (req.displacement, req.rotation))
    return ControlCommandResponse(True)

def control_node_init():
    rospy.init_node(NAME)
    s = rospy.Service('control_service', ControlCommand, control_command_callback)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    rospy.loginfo("Starting control_node service")
    control_node_init()