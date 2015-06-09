#!/usr/bin/env python

import rospy, socket, atexit, re
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

'''
This module is responsible for reporting GPS information from the Jaguar robot to the 
 ROS environment.

MODULE TODO LIST: 
 - 
'''

__authors__ = ["Alex Lalejini", "Ryan Smith", "Dexter Duckworth"]

#########################################
# Constants
#########################################
# Networking
DEFAULT_GPS_IP = "192.168.0.71"
DEFAULT_GPS_PORT = 10002
BUFFER = 128
# Topic(s)
DEFAULT_GPS_TOPIC = "jaguar_gps"
#########################################

class GPS_Reporter(object):

    def __init__(self):
        '''
        GPS Reporter contructor.
        '''
        # Register as ROS node.
        rospy.init_node("gps_reporter")
        # Register an exit handler.
        atexit.register(self._exit_handler)

        # Setup networking
        # Load GPS ip and port from parameter server.
        self.imu_ip = rospy.get_param("sensors/gps/ip", DEFAULT_GPS_IP)
        self.imu_port = rospy.get_param("sensors/gps/port", DEFAULT_GPS_PORT)
        # Create GPS comms socket (TCP communication protocol)
        self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Attempt to connect
        rospy.loginfo("Trying to connect to GPS at " + str(self.gps_ip) + " on port " + str(self.gps_port))
        while not rospy.is_shutdown():
            try:
                self.gps_sock.connect((self.gps_ip, self.gps_port))
            except:
                rospy.logerr("Failed to connect to GPS.  Will continue trying.")
                rospy.sleep(3)
            else:
                rospy.loginfo("Successfully connected to GPS.")

        # Load topic name(s)
        self.gps_topic = rospy.get_param("sensors/gps/topic", DEFAULT_GPS_TOPIC)
        # Create necessary ROS publishers
        self.gps_pub = rospy.Publisher(self.gps_topic, NavSatFix, queue_size = 10)

    def parse_gps(data):
        '''
        Given raw data from Jaguar GPS socket, parse and return NavSatFix message.
        If bad/incomplete data, return None
        '''
        # use regex to parse
        gpgga_hit = re.search('^\$(GPGGA.+?)\r\n', data)
        gprmc_hit = re.search('^\$(GPRMC.+?)\r\n', data)
        if gpgga_hit and gprmc_hit:
            gpgga_str = gpgga_hit.group(0)
            gprmc_str = gprmc_hit.group(0)

        else:
            return None

        pass

    def run(self):
        '''
        Main loop run function.
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Grab a chunk of data
            data = self.gps_sock.recv(BUFFER)
            gps_msg = self.parse_gps(data)
            self.gps_pub.publish(gps_msg)
            rate.sleep()

    def _exit_handler(self):
        '''
        This function runs on module exit.
        '''
        exit()

if __name__ == "__main__":
    pass

