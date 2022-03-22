#!/usr/bin/env python

from painted.srv import *
import rospy
from math import *
from math import pi, radians

NAME = 'transform_node'


class transform_node():
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.transform_geojson_service = rospy.Service(
            'transform_geojson_service', TransformToGeoJsonCommand, self.transform_to_geojson_callback)

        self.transform_robot_service = rospy.Service(
            'transform_robot_service', TransformToRobotCommand, self.transform_to_robot_callback)

        # Initial parameters
        self.geojson_x = -3.199559805791
        self.geojson_y = 55.9380467435949

        self.division_x = 62345.65
        self.division_y = 111139
        self.scale = 100

        self.irl_x = -0.095
        self.irl_y = -0.075

        self.log_info("Starting %s service" % (NAME))

    def transform_to_geojson_callback(self, req):
        x = req.x
        y = req.y
        return self.relative_point_real_to_geo(x, y)

    def transform_to_robot_callback(self, req):
        x = req.x
        y = req.y
        return self.relative_point_geo_to_real(x, y)

    def relative_point_real_to_geo(self, x_coord, y_coord):
        change_in_x = (x_coord - self.irl_x) * self.scale
        change_in_y = (y_coord - self.irl_y) * self.scale
        # get the relative x, y movement
        x_movement = change_in_x / self.division_x
        y_movement = change_in_y / self.division_y
        # return point in geojson world.
        return (self.geojson_x + x_movement), (self.geojson_y + y_movement)

    def relative_point_geo_to_real(self, x_coord, y_coord):
        change_in_x = (x_coord - self.geojson_x) * self.division_x
        change_in_y = (y_coord - self.geojson_y) * self.division_y
        # get the relative x, y movement
        x_movement = change_in_x / self.scale
        y_movement = change_in_y / self.scale
        # return point in robot world.
        return (self.irl_x + x_movement), (self.irl_y + y_movement)

    def shutdown(self):
        # stop turtlebot
        self.log_info("Stopping %s node" % (NAME))

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting %s service" % (NAME))
    transform_node()
    rospy.spin()
