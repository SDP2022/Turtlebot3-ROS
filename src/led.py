#!/usr/bin/env python

from painted.srv import *
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from math import *
from math import pi, radians
from nav_msgs.msg import Odometry
from std_msgs.msg import String
try:
    from gpiozero import LED
    SIM_ENV = False
    print('Turtlebot environment detected, enable LED api')
except ImportError:
    print('Sim environment detected, disable LED api')
    SIM_ENV = True

NAME = 'led_node'


class led_node():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.len_node_service = rospy.Service(
            'led_service', LEDCommand, self.LED_command_callback)

        # Initial LED parameters
        if not SIM_ENV:
            self.led = LED(14)
            self.led.off()
        self.led_status = False  # inital LED status is off
        self.log_info("Starting %s service" % (NAME))

        if SIM_ENV:
            rospy.logwarn("[{0}]{1}".format(NAME, 'SIM MODE ON'))
        else:
            rospy.logwarn("[{0}]{1}".format(NAME, 'SIM MODE OFF'))

    def LED_command_callback(self, req):
        command_led = req.led
        self.log_info("command_led=%s self.led_status=%s" %
                      (command_led, self.led_status))
        if command_led == self.led_status:
            self.log_info("Statue same, wont execute")
            return False
        if command_led:
            self.led.on()
        else:
            self.led.off()
        return True

    def shutdown(self):
        # stop turtlebot, reset
        self.log_info("Stopping %s node" % (NAME))
        if self.led_status and not SIM_ENV:
            self.led.off()
            self.log_info(" LED off success")
        else:
            self.log_info("LED already off, no resetting")

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    led_node()
    rospy.spin()
