#!/usr/bin/env python

from painted.srv import *
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from math import *
from math import pi, radians
from nav_msgs.msg import Odometry
from std_msgs.msg import String
try:
    from motors import Motors
    SIM_ENV = False
    print('Turtlebot environment detected, enable motor api')
except ImportError:
    print('Sim environment detected, disable motor api')
    SIM_ENV = True

NAME = 'pen_node'


class pen_node():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.pen_node_service = rospy.Service(
            'pen_service', PenCommand, self.pen_command_callback)

        # Initial motor parameters
        if not SIM_ENV:
            self.motor = Motors()
        self.motor_id = 2
        self.speed = 100  # forward = positive, backwards = negative
        self.pen_down_status = False  # inital pen status is up
        self.pen_running = False # For perventing multiple pen executing 
        self.log_info("Starting %s service" % (NAME))

        if SIM_ENV:
            rospy.logwarn("[{0}]{1}".format(NAME, 'SIM MODE ON'))
        else:
            rospy.logwarn("[{0}]{1}".format(NAME, 'SIM MODE OFF'))

    def pen_command_callback(self, req):
        if self.pen_running is True:
            return False
        else:
            self.pen_running = True
        command_pen_status = req.pen_down
        self.log_info("command_pen_status=%s self.pen_down_status=%s" %
                      (command_pen_status, self.pen_down_status))
        if command_pen_status == self.pen_down_status:
            self.log_info("Statue same, wont execute")
            self.pen_running = False
            return False
        if command_pen_status:
            self.pen_down()
        else:
            self.pen_up()
        self.pen_running = False
        return True

    def pen_down(self):
        r = rospy.Rate(2.2)
        self.log_info('Execute pen_down with id=%i speed=%i' %
                      (self.motor_id, -self.speed))
        if not SIM_ENV:
            self.motor.move_motor(self.motor_id, self.speed)
            r.sleep()
            self.motor.stop_motors()
        self.pen_down_status = True
        self.log_info('Execute pen_down success, pen_down_status=%s' %
                      (self.pen_down_status))

    def pen_up(self):
        r = rospy.Rate(2.2)
        self.log_info('Execute pen_up with id=%i speed=%i' %
                      (self.motor_id, self.speed))
        if not SIM_ENV:
            self.motor.move_motor(self.motor_id, -self.speed)
            r.sleep()
            self.motor.stop_motors()
        self.pen_down_status = False
        self.log_info('Execute pen_up success, pen_down_status=%s' %
                      (self.pen_down_status))

    def shutdown(self):
        # stop turtlebot, reset
        self.log_info("Stopping %s node" % (NAME))
        if self.pen_down_status:
            self.pen_up()
            self.log_info("Reset pen up success")
        else:
            self.log_info("Pen already up, no resetting")

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    pen_node()
    rospy.spin()
