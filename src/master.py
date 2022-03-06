#!/usr/bin/env python3
from distutils.archive_util import make_archive
import rospy
import json
from std_msgs.msg import String
# from painted.srv import *

class master:
    def __init__(self) -> None:
        rospy.init_node('master', anonymous=True)
        self.pub_web = rospy.Publisher('web_messages', String, queue_size=10)

        self.pub_web.publish(self.make_web_message("success", "hello world " + str(rospy.get_time())))

        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.pub_web.publish(self.make_web_message("success", "hello world " + str(rospy.get_time())))
        #     rate.sleep()

        rospy.loginfo("test")


        self.sub_job = rospy.Subscriber('start_job', String, self.job_callback)



    def job_callback(self, data):

        #testing a hypothetical situation
        #not currently using messages, just sending string of JSON
        rospy.loginfo("job received")
        self.pub_web.publish(self.make_web_message("success", "hello world " + str(rospy.get_time())))


        user_data = json.loads(data.data)
        geoJSON = user_data['geoJSON'] #eg in future
        rospy.loginfo(data.data)
    
        self.pub_web.publish( self.make_web_message("info", "Job received"))
        rospy.sleep(3)
        self.pub_web.publish( self.make_web_message("success", "Job will begin shortly"))
        rospy.sleep(3)
        self.pub_web.publish( self.make_web_message("info", "Imagine the robot is now mapping"))
        rospy.sleep(3)
        self.pub_web.publish( self.make_web_message("info", "Imagine the robot is now painting"))
        rospy.sleep(3)
        self.pub_web.publish(self.make_web_message("success", "Imagine the job is now complete!"))
        
    def make_web_message(self, alert_type, message):
        return json.dumps({
            "alert_type" : alert_type,
            "message" : message
        })

if __name__ == "__main__":
    rospy.loginfo("LEEEEEROY JENKINS")
    rospy.loginfo("Starting master_node service")
    master()
    rospy.spin()