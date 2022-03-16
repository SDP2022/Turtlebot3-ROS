#!/usr/bin/env python
from distutils.archive_util import make_archive
import rospy
import json
from std_msgs.msg import String
from painted.srv import *

class master:
    def __init__(self):
        rospy.init_node('master', anonymous=True)
        self.pub_web = rospy.Publisher('web_messages', String, queue_size=10)
        self.pub_web.publish(self.make_web_message("success", "PaintBot is ready to go! "))
        self.sub_job = rospy.Subscriber('start_job', String, self.job_callback)
        rospy.wait_for_service('execute_service')
        self.log_info("Starting %s service" % (NAME))

    def job_callback(self, data):
        #testing a hypothetical situation
        #not currently using messages, just sending string of JSON
        self.log_info("Job recieved")
        self.pub_web.publish( self.make_web_message("success", "Job has been received and will be processed."))

        user_data = json.loads(data.data)
        geoJSON = user_data['geoJSON'] #eg in future
        rospy.loginfo(data.data)
        #TODO Feed geojson into execution
        self.log_info("Job planning success")
        self.pub_web.publish( self.make_web_message("success", "Job planning successs."))

        self.log_info("Job executing")
        self.pub_web.publish( self.make_web_message("info", "PaintBot is now executing the job."))
        self.execute_command()

        self.log_info("Job success")
        self.pub_web.publish(self.make_web_message("success", "Job is now complete!"))
        
    def make_web_message(self, alert_type, message):
        return json.dumps({
            "alert_type" : alert_type,
            "message" : message
        })
    
    def execute_command(self):
        try:
            execute_command = rospy.ServiceProxy('execute_service', ExecuteCommand)
            self.log_info("Requesting execute_service")
            resp1 = execute_command(True)
            return resp1.status
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
            self.pub_web.publish(self.make_web_message("error", "An error has occurred in the execution command :("))

    def log_info(self, message):
        return rospy.loginfo("[{0}]{1}".format(NAME, message))

if __name__ == "__main__":
    rospy.loginfo("Starting master_node service")
    master()
    rospy.spin()