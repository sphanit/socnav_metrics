#!/usr/bin/env python3

# What should be the message structures for
# 1) Getting the data via topic
# 2) Publishing the data via topic
# 
# Are we making any log files?
# I think, we should have a unified visualisation for all API --> Should be developed separately.
# pedestrians or humans?
# What is the representation for the obstacles?
# Current time or simulator time?

import rospy
import numpy as np
from std_msgs.msg import String
from ros_api.msg import MetricData
        

class RosAPI(object):
    def __init__(self):
        rospy.init_node('socnav_api')
        self.pedestrians = None
        self.robot = None
        self.obstacles = None ##### Do we really need it?
        self.time = 0.0
        self.goal = None
        self.metrics = MetricData()
        
        # rospy.Subscriber("~agents_data", <<AgentsMsg>>, self.parse_data)
        self.metrics_pub = rospy.Publisher('~metrics', MetricData, queue_size = 10)
        rospy.Timer(rospy.Duration(0.1), self.publish_metrics)
        rospy.spin()

    def publish_metrics(self, event):
        self.metrics.time_stamp = rospy.Time.now()
        self.metrics.metrics = "collisons: 2, min_hr_dist: 0.7"
        self.metrics_pub.publish(self.metrics)

    def parse_data(self, msg):
        self.pedestrians = msg.pedestrians
        self.robot = msg.robot
        self.obstacles = msg

    def calculate_metrics(self):
        self.time = rospy.get_time()

if __name__ == '__main__':
    rosapi = RosAPI()
