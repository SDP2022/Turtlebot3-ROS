#!/usr/bin/env python

from painted.srv import *
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from math import *
from math import pi, radians
from nav_msgs.msg import Odometry
from std_msgs.msg import String
try:
    from gpiozero import Buzzer
    SIM_ENV = False
    print('Turtlebot environment detected, enable Buzzer api')
except ImportError:
    print('Sim environment detected, disable Buzzer api')
    SIM_ENV = True

NAME = 'buzzer_node'


class buzzer_node():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.len_node_service = rospy.Service(
            'buzzer_service', BuzzerCommand, self.buzzer_command_callback)

        # Initial Buzzer parameters
        if not SIM_ENV:
            self.buzzer = Buzzer(4)
            self.buzzer.off()
        self.log_info("Starting %s service" % (NAME))

        if SIM_ENV:
            rospy.logwarn("[{0}]{1}".format(NAME, 'SIM MODE ON'))
        else:
            rospy.logwarn("[{0}]{1}".format(NAME, 'SIM MODE OFF'))

    def buzzer_command_callback(self, req):
        self.buzzer.beep(n=1)
        return True

    def shutdown(self):
        # stop turtlebot, reset
        self.log_info("Stopping %s node" % (NAME))
        if not SIM_ENV:
            self.buzzer.off()

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    buzzer_node()
    rospy.spin()
