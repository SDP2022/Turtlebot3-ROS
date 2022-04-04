#!/usr/bin/env python

import os
os.popen('source /opt/ros/kinetic/setup.bash')
os.popen('source /home/pi/catkin_ws/devel/setup.bash')
from painted.srv import *
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from math import *
from math import pi, radians
from nav_msgs.msg import Odometry
from std_msgs.msg import String
try:
    from gpiozero import LED
    from grove.grove_ws2813_rgb_led_strip import GroveWS2813RgbStrip
    from rpi_ws281x import Color
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
            # self.led = LED(14)
            self.led_strip = GroveWS2813RgbStrip(18, 60)
            self.colorWipe(self.led_strip, Color(0,0,0))
            # self.led.off()
        self.led_status = False  # inital LED status is off
        self.log_info("Starting %s service" % (NAME))

        self.blue = Color(0,143,255)
        self.yellow = Color(255,255,0)
        self.green = Color(41,255,0)
        self.red = Color(255,0,43)

        if SIM_ENV:
            rospy.logwarn("[{0}]{1}".format(NAME, 'SIM MODE ON'))
        else:
            rospy.logwarn("[{0}]{1}".format(NAME, 'SIM MODE OFF'))

    def LED_command_callback(self, req):
        command_led = req.led
        self.log_info("command_led=%s self.led_status=%s" %
                      (command_led, self.led_status))
        if command_led == 'red':
            self.colorWipe(self.led_strip, self.red)
        elif command_led == 'yellow':
            self.colorWipe(self.led_strip, self.yellow)
        elif command_led == 'green':
            self.colorWipe(self.led_strip, self.green)
        elif command_led == 'blue':
            self.colorWipe(self.led_strip, self.blue)
        else:
            self.colorWipe(self.led_strip, Color(0,0,0))
        return True
    
    def colorWipe(self, strip, color, wait_ms=50):
        """Wipe color across display a pixel at a time."""
        for i in range(strip.numPixels()*2):
            strip.setPixelColor(i, color)
            strip.show()
            rospy.sleep(wait_ms/1000.0)

    def shutdown(self):
        # stop turtlebot, reset
        self.log_info("Stopping %s node" % (NAME))
        self.colorWipe(self.led_strip, Color(0,0,0))

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    led_node()
    rospy.spin()
