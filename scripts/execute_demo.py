#!/usr/bin/env python

import sys
import os
from turtle import Pen, pen

import rospy

from painted.srv import *

def control_command_client(x, y):
    rospy.wait_for_service('control_service')
    
    try:
        control_command = rospy.ServiceProxy('control_service', ControlCommand)
        print("Requesting displacement=%s rotation=%s"%(x, y))
        # simplified style
        resp1 = control_command(x, y)
        # formal style
        # resp2 = control_command.call(ControlCommandRequest(x, y)) 
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def pen_command_client(pen_status):
    rospy.wait_for_service('pen_service')
    try:
        pen_command = rospy.ServiceProxy('pen_service', PenCommand)
        print("Requesting pen_status=%s"%(pen_status))
        resp1 = pen_command(pen_status)
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    
    argv = rospy.myargv()
    if len(argv) == 3:
        try:
            x = int(argv[1])
            y = int(argv[2])
            print("displacement=%s rotation=%s status= %s"%(x, y, control_command_client(x, y)))
        except Exception as e:
            print('control service call error ' + '%s [x y]'%sys.argv[0])
            print(e)
            sys.exit(1)
    elif len(argv) == 2:
        try:
            pen_status = bool(eval((argv[1])))
            print("pen_status=%s status=%s"%(pen_status, pen_command_client(pen_status)))
        except Exception as e:
            print('pen service call error ' + '%s [pen_status]'%sys.argv[1])
            print(e)
            sys.exit(1)
    else:
        print('incorrect argv length' + str(len(argv)))
        sys.exit(1)

