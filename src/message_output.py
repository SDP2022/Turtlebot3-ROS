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
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import copy
import json


class message_output():
    def __init__(self):
        rospy.init_node('momessage_outputve', anonymous=True)
        self.ubscriber = rospy.Subscriber("/map", OccupancyGrid, self.listener_callback)
        # raise rospy.ROSInterruptException

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stopping move node")
        rospy.sleep(1)
        raise rospy.ROSInterruptException

    def listener_callback(self, msg):
        map_obj = Map(msg.data, msg.info.height, msg.info.width, \
            msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y, \
            msg.info.origin.position.z, msg.info.origin.orientation.x, msg.info.origin.orientation.y, \
            msg.info.origin.orientation.z, msg.info.origin.orientation.w)
        json_string = json.dumps(map_obj.__dict__)
        # Using a JSON string
        with open('json_data.json', 'w') as outfile:
            json.dump(json_string, outfile)
        # print(json_string)

class Map:
    def __init__(self, data, height, width, resolution, origin_pos_x, origin_pos_y, origin_pos_z, origin_ori_x, origin_ori_y, origin_ori_z, origin_ori_w):
        self.data = data
        self.height = height
        self.width = width
        self.resolution = resolution
        self.origin_pos_x = origin_pos_x
        self.origin_pos_y = origin_pos_y
        self.origin_pos_z = origin_pos_z
        self.origin_ori_x = origin_ori_x
        self.origin_ori_y = origin_ori_y
        self.origin_ori_z = origin_ori_z
        self.origin_ori_w = origin_ori_w


		

if __name__ == '__main__':
    try:
        message_output()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")