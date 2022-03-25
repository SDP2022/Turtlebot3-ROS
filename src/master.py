#!/usr/bin/env python
from distutils.archive_util import make_archive
from sys import path_hooks
import rospy
import json
from std_msgs.msg import String
from painted.msg import State, Job
from painted.srv import *
import json
from geo_parser import GeoParser
from final_pathfinding import Pathfinder
from executelist_parser import ExecuteListParser

NAME = 'master_node'


class master:
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)
        self.pub_web = rospy.Publisher('web_messages', String, queue_size=10)
        self.pub_web.publish(self.make_web_message(
            "success", "PaintBot is ready to go! "))
        self.sub_job = rospy.Subscriber('start_job', String, self.job_callback)
        self.state_pub = rospy.Publisher('/state', State, queue_size=10)
        self.job_id_ = None
        self.as_state_ = State()
        self.as_state_.as_state = State().AS_OFF
        self.state_sub = rospy.Subscriber('/state', State, self.state_cb)

        rospy.wait_for_service('execute_service')
        self.log_info("Starting %s service" % (NAME))

    def state_cb(self, msg):
        self.setState(msg.as_state)

    def setState(self, state_):
        if (state_ == State().AS_READY):
            self.as_state_ = State().AS_READY
        if (state_ == State().AS_RUNNING):
            self.as_state_ = State().AS_RUNNING
        if (state_ == State().AS_REQUEST_PAUSE):
            self.as_state_ = State().AS_REQUEST_PAUSE
        if (state_ == State().AS_PAUSED):
            self.as_state_ = State().AS_PAUSED
        if (state_ == State().AS_FINISHED):
            self.as_state_ = State().AS_FINISHED

    def queryState(self):
        return self.as_state_

    def job_callback(self, msg):
        # testing a hypothetical situation
        # not currently using messages, just sending string of JSON
        self.log_info("Job recieved")
        self.pub_web.publish(self.make_web_message(
            "success", "Job has been received and will be processed."))

        # preliminary implementation of job msg handling
        self.job_id_ = msg.job_id
        user_data = json.loads(msg.job_data)
        geoJSON = user_data['geoJSON']  # eg in future
        self.log_info('msg.job_data=%s' % (msg.job_data))
        geojson_path = self.save_geojson_file((geoJSON))
        # Feed geojson into execution
        drawing_points = GeoParser.parser_file_red(geojson_path)
        obsolete_list = GeoParser.parser_file_blue(geojson_path)
        self.log_info('drawing_points=%s}' % (drawing_points))
        self.log_info('obsolete_list=%s}' % (obsolete_list))
        start_position = [30.41942059993744, 49.003321098987215]
        point_list = Pathfinder(start_position, drawing_points, obsolete_list)
        self.log_info('Pointlist=%s}' % (point_list))
        to_executelist_parser = ExecuteListParser(
            start_position[0], start_position[1])
        execute_list = to_executelist_parser.to_execute_list(point_list)
        self.log_info(
            'Job planning execute list success execute_list=%s}' % (execute_list))
        self.pub_web.publish(self.make_web_message(
            "success", "Job planning successs."))
        self.state_pub.publish(State(1))

        # execution line by line
        self.log_info("Job executing")
        self.pub_web.publish(self.make_web_message(
            "info", "PaintBot is now executing the job."))
        self.state_pub.publish(State(2))
        for execute_path in execute_list:
            next_x = execute_path[0][0]
            next_y = execute_path[0][1]
            direction = execute_path[1]
            distance = execute_path[2]
            self.log_info('executing x=%s y=%s diretion=%s distance=%s')
            self.execute_command(next_x, next_y, direction, distance)

        self.log_info("Job success")
        self.pub_web.publish(self.make_web_message(
            "success", "Job is now complete!"))
        self.state_pub.publish(State(5))

    def make_web_message(self, alert_type, message):
        return json.dumps({
            "alert_type": alert_type,
            "message": message
        })

    def execute_command(self, x, y, direction, distance):
        try:
            execute_command = rospy.ServiceProxy(
                'execute_service', ExecuteCommand)
            self.log_info("Requesting execute_service")
            resp1 = execute_command(x, y, direction, distance)
            return resp1.status
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            self.pub_web.publish(self.make_web_message(
                "error", "An error has occurred in the execution command :("))

    def save_geojson_file(self, json_string):
        json_path = 'task.json'
        with open(json_path, 'w') as f:
            json.dump(json_string, f)
        self.log_info('GeoJson saved path=%s' % (json_path))
        return json_path

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))


if __name__ == "__main__":
    rospy.loginfo("Starting master_node service")
    master()
    rospy.spin()
