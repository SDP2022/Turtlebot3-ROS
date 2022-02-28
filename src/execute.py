#!/usr/bin/env python

import sys
import os

import rospy

from painted.srv import *

def control_command_client(x, y):
    rospy.wait_for_service('control_service')
    
    try:
        control_command = rospy.ServiceProxy('control_service', ControlCommand)
        print("Requesting %s+%s"%(x, y))
        # simplified style
        resp1 = control_command(x, y)
        # formal style
        # resp2 = control_command.call(ControlCommandRequest(x, y)) 
        return resp1.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    
    argv = rospy.myargv()
    if len(argv) == 1:
        import random
        x = random.randint(-50000, 50000)
        y = random.randint(-50000, 50000)
    elif len(argv) == 3:
        try:
            x = int(argv[1])
            y = int(argv[2])
        except:
            print(usage())
            sys.exit(1)
    else:
        print(usage())
        sys.exit(1)
    print("%s + %s = %s"%(x, y, control_command_client(x, y)))
