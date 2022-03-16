#!/usr/bin/env python

from re import S, T
from painted.srv import *
import rospy
import json
import math
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

NAME = 'execute_node'


class execute_node():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.execute_service = rospy.Service(
            'execute_service', ExecuteCommand, self.execute_command_callback)
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.wait_for_service('pen_service')
        rospy.wait_for_service('control_service')
        self.log_info("Starting %s service" % (NAME))

    def execute_command_callback(self, req):
        self.log_info('Starting execute')
        return True

    def control_command(self, displacement, rotation):
        try:
            control_command = rospy.ServiceProxy('control_service', ControlCommand)
            print("Requesting displacement=%s rotation=%s"%(displacement, rotation))
            resp1 = control_command(displacement, rotation)
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def pen_command(self, pen_status):
        try:
            pen_command = rospy.ServiceProxy('pen_service', PenCommand)
            print("Requesting pen_status=%s"%(pen_status))
            resp1 = pen_command(pen_status)
            d = rospy.Duration(2, 0)
            rospy.sleep(d)
            return resp1.status
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def goto(self, x, y, direction):
        pos = {'x': x, 'y' : y}
        quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : direction}
        self.log_info('Requesting goto pos={0} quat={1}'.format(pos, quat))
        # Send a goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        # Start moving
        self.move_base.send_goal(goal)
        self.log_info('Goal send')
        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))
        self.log_info('Waiting result')
        state = self.move_base.get_state()
        result = False
        self.log_info('Goal state=%s' % (str(state)))
        if success and state == GoalStatus.SUCCEEDED:
            self.log_info('Goal success')
            result = True
        else:
            self.log_info('Goal Failed')
            self.move_base.cancel_goal()
        return result

    def shutdown(self):
        # stop turtlebot, reset
        self.log_info("Stopping %s node" % (NAME))

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    execute_node()
    rospy.spin()
