#!/usr/bin/env python
from ast import While
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
        self.sub_job = rospy.Subscriber('start_job', Job, self.job_callback)
        self.state_pub = rospy.Publisher('/state', State, queue_size=10)
        self.job_id_ = '-1'
        self.as_state_ = State(0)
        self.as_state_.as_state = State().AS_OFF
        self.state_sub = rospy.Subscriber('/state', State, self.state_cb)
        self.job_status_service = rospy.Service(
            'job_status_service', JobStatusCommand, self.job_status_command_callback)

        rospy.wait_for_service('execute_service')
        self.log_info("Starting %s service" % (NAME))

    def state_cb(self, msg):
        self.setState(msg.as_state)

    def setState(self, state_):
        if (state_ == State().AS_READY):
            self.as_state_ = State(State().AS_READY)
        if (state_ == State().AS_RUNNING):
            self.as_state_ = State(State().AS_RUNNING)
        if (state_ == State().AS_REQUEST_PAUSE):
            self.as_state_ = State(State().AS_REQUEST_PAUSE)
        if (state_ == State().AS_PAUSED):
            self.as_state_ = State(State().AS_PAUSED)
        if (state_ == State().AS_FINISHED):
            self.as_state_ = State(State().AS_FINISHED)

    def job_status_command_callback(self, req):
        # print(type(self.as_state_.as_state))
        return self.job_id_, self.as_state_.as_state

    def job_callback(self, msg):
        # testing a hypothetical situation
        # not currently using messages, just sending string of JSON
        self.state_pub.publish(State(1))
        self.log_info("Job Recieved")
        self.pub_web.publish(self.make_web_message(
            "success", "Job has been received and will be processed."))

        # preliminary implementation of job msg handling
        self.job_id_ = msg.job_id
        self.log_info('Job ID=%s' % (self.job_id_))
        self.log_info('Job Data=%s' % (msg.job_data))
        user_data = json.loads(msg.job_data)
        # Feed geojson into execution
        geo_parser = GeoParser()
        drawing_points = geo_parser.parser_file_red(user_data)
        obsolete_list = geo_parser.parser_file_blue(user_data)
        self.log_info('Drawing_points=%s}' % (drawing_points))
        self.log_info('Obsolete_list=%s}' % (obsolete_list))
        start_position = [-3.200362642, 55.937597084]
        point_finder = Pathfinder(start_position, drawing_points, obsolete_list)
        point_list = point_finder.all_together_now()
        self.log_info('Pointlist=%s}' % (point_list))
        execute_list = ExecuteListParser(point_list)
        self.log_info(
            'Job planning execute list success execute_list=%s' % (execute_list))
        self.log_info('len(execute_list)=%s' %(len(execute_list)))
        
        self.pub_web.publish(self.make_web_message(
            "success", "Job planning successs."))

        # execution line by line
        self.log_info("Job executing")
        self.pub_web.publish(self.make_web_message(
            "info", "PaintBot is now executing the job."))
        self.state_pub.publish(State(2))
        execute_path_index = 0
        while True:
            if self.as_state_ == State(3):
                self.log_info('Job Pause')
                d = rospy.Duration(5, 0)
                self.pub_web.publish(self.make_web_message(
                    "warning", "Job is now paused. Please resume when ready!"))
                rospy.sleep(d)
                continue
            elif execute_path_index == len(execute_list):
                break
            else:
                self.log_info(execute_path_index)
                inc_by = 1
                next_x = execute_list[execute_path_index][0][0]
                next_y = execute_list[execute_path_index][0][1]
                direction = execute_list[execute_path_index][1]
                distance = execute_list[execute_path_index][2]
                self.log_info('Executing x=%s y=%s direction=%s distance=%s' % (next_x, next_y, direction, distance))
                self.pub_web.publish(self.make_web_message(
                    "info", 'Executing x=%s y=%s direction=%s distance=%s' % (next_x, next_y, direction, distance)))
                execute_command = self.execute_command(next_x, next_y, direction, distance)
                if not execute_command:
                    self.state_pub.publish(State(3))
                    inc_by = 0
                execute_path_index += inc_by
        self.log_info("Job success")
        self.pub_web.publish(self.make_web_message(
            "success", "Job is now complete!"))
        self.state_pub.publish(State(5))
        self.job_id_ = '-1'
        self.as_state_ = State(0)

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
