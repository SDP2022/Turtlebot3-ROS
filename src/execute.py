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
from painted.msg import State
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

NAME = 'execute_node'


class execute_node():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.execute_service = rospy.Service(
            'execute_service', ExecuteCommand, self.execute_command_callback)
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.state_pub = rospy.Publisher('/state', State, queue_size=10)
        self.state_sub = rospy.Subscriber('/state', State, self.state_cb)
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.wait_for_service('pen_service')
        rospy.wait_for_service('control_service')
        rospy.wait_for_service('led_service')
        rospy.wait_for_service('buzzer_service')
        self.log_info("Starting %s service" % (NAME))

    def state_cb(self, msg):
        if msg.as_state == State().AS_PAUSED:
            self.buzzer_command(2)
    
    def execute_command_callback(self, req):
        # start_x = req.start_x
        # start_y = req.start_y
        start_y = req.start_x
        start_x = req.start_y
        direction = req.direction
        distance = req.distance
        self.log_info('Starting execute once. starting_pos=%s,%s direction=%s distance=%s' % (start_x, start_y, direction, distance))
        # goto starting position
        self.led_command('blue')
        self.buzzer_command(1)
        goto_result = self.goto(start_x, start_y, direction)
        self.buzzer_command(1)
        if not goto_result:
            self.led_command('red')
            return False
        self.led_command('yellow')
        # start drawing)
        self.buzzer_command(2)
        self.pen_command(True)
        control_result = self.control_command(distance, 0)
        self.pen_command(False)
        self.buzzer_command(2)
        if not control_result:
            self.led_command('red')
            return False
        self.led_command('green')
        return True

    def control_command(self, displacement, rotation):
        try:
            control_command = rospy.ServiceProxy('control_service', ControlCommand)
            self.log_info("Requesting displacement=%s rotation=%s"%(displacement, rotation))
            resp1 = control_command(displacement, rotation)
            return resp1.status
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def pen_command(self, pen_status):
        try:
            pen_command = rospy.ServiceProxy('pen_service', PenCommand)
            self.log_info("Requesting pen_status=%s"%(pen_status))
            resp1 = pen_command(pen_status)
            d = rospy.Duration(2, 0)
            rospy.sleep(d)
            return resp1.status
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def led_command(self, led_status):
        try:
            led_command = rospy.ServiceProxy('led_service', LEDCommand)
            self.log_info("Requesting LED_status=%s"%(led_status))
            resp1 = led_command(led_status)
            return resp1.status
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def buzzer_command(self, buzzer_beep):
        try:
            buzzer_command = rospy.ServiceProxy('buzzer_service', BuzzerCommand)
            self.log_info("Requesting Buzzer_status=%s"%(buzzer_beep))
            resp1 = buzzer_command(buzzer_beep)
            return resp1.status
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def goto(self, x, y, direction):
        pos = {'x': x, 'y' : -y}
        z, w = self.angle_to_quat(direction)
        quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : z, 'r4' : w}
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
        self.state_pub.publish(State(2))
        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))
        self.log_info('Waiting result')
        state = self.move_base.get_state()
        result = False
        self.log_info('Goal state=%s' % (str(state)))
        if success and state == GoalStatus.SUCCEEDED:
            self.log_info('Goal success')
            self.state_pub.publish(State(5))
            result = True
        else:
            self.log_info('Goal Failed')
            self.move_base.cancel_goal()
        return result

    def shutdown(self):
        # stop turtlebot, reset
        self.log_info("Stopping %s node" % (NAME))
        self.state_pub.publish(State(0))

    def angle_to_quat(self, angle):
        q = quaternion_from_euler(0,0,angle)
        z = q[2]
        w = q[3]
        return z, w

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    execute_node()
    rospy.spin()
