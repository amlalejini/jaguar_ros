#!/usr/bin/env python

import rospy

'''
This module is responsible for reporting GPS information from the Jaguar robot to the 
 ROS environment.

MODULE TODO LIST: 
 - 
'''

#########################################
# Constants
#########################################
# Topic(s)
DEFAULT_GPS_TOPIC = "jaguar_gps"
#########################################

class GPS_Reporter(object):

    def __init__(self):
        '''
        GPS Reporter contructor.
        '''
        rospy.init_node("gps_reporter")

        # Load topic name(s)
        self.gps_topic = rospy.get_param("sensors/gps/topic", DEFAULT_GPS_TOPIC)
        # Create necessary ROS publishers
        self.gps_pub = rospy.Publisher(self.gps_topic, None, queue_size = 10)

    def run(self):
        '''
        Main loop run function.
        '''
        pass

if __name__ == "__main__":
    pass

