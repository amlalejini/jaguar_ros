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
BUFFER = 2056
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

        self.current_est_heading = None
        self.current_gprmc = None
        self.current_gpgga = None

        # Register an exit handler.
        atexit.register(self._exit_handler)
        # Setup networking
        # Load GPS ip and port from parameter server.
        self.gps_ip = rospy.get_param("sensors/gps/ip", DEFAULT_GPS_IP)
        self.gps_port = rospy.get_param("sensors/gps/port", DEFAULT_GPS_PORT)
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
                break

        # Load topic name(s)
        self.gps_topic = rospy.get_param("sensors/gps/topic", DEFAULT_GPS_TOPIC)
        # Create necessary ROS publishers
        self.gps_pub = rospy.Publisher(self.gps_topic, NavSatFix)#, queue_size = 10)

    def parse_gps(self, data):
        '''
        Given raw data from Jaguar GPS socket, parse and return NavSatFix message.
        If bad/incomplete data, return None
        '''
        # use regex to parse
        gpgga_hit = re.search('^\$(GPGGA.+?)\r\n', data)
        gprmc_hit = re.search('^\$(GPRMC.+?)\r\n', data)
        print("=====")
        print(data)
        # TODO: Might not even want to use GPRMC?
        if gpgga_hit:
            self.current_gpgga = gpgga_hit.group(0)
        if gprmc_hit:
            self.current_gprmc = gprmc_hit.group(0)
        if self.current_gpgga and self.current_gprmc:
            gpgga = self.current_gpgga.split(",")
            gprmc = self.current_gprmc.split(",")
            self.current_gpgga = None
            self.current_gprmc = None
            nav_msg = NavSatFix()
            # Set header information 
            time = gpgga[1]
            hrs = float(time[0:1])
            mins = float(time[2:3])
            secs = float(time[4:5])
            nav_msg.header.stamp = rospy.Time.from_sec(hrs * 3600 + mins * 60 + secs)
            nav_msg.header.frame_id = "gps"
            # Set GPS Status
            status = NavSatStatus()
            status.status = -1 if int(gpgga[6]) == 0 else 0
            nav_msg.status = status
            # Set longitude and latitude http://stackoverflow.com/questions/28174127/converting-strange-lat-lng-format-to-decimal
            lat_str = gppga[2]
            lon_str = gppga[4]
            lat_degs = float(lat_str[:2]) + (float(lat_str[2:]) / 60.0)
            lon_degs = float(lon_str[:3]) + (float(lon_str[3:]) / 60.0)
            nav_msg.latitude = -1 * lat_degs if gpgga[3] == "S" else lat_degs
            nav_msg.longitude = -1 * lon_degs if gpgga[5] == "W" else lon_degs
            # Set covariance type to unknown
            nav_msg.position_covariance_type = 0
            # Get estimated heading (not part of standard ROS navsatfix message)
            try:
                heading = float(gprmc[8])
            except:
                heading = float("NaN")
            else:
                while heading < -180.0:
                    heading += 360.0
                while heading > 180.0:
                    heading -= 360.0
            finally:
                self.current_est_heading = heading

            return nav_msg
        else:
            return None


    def run(self):
        '''
        Main loop run function.
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Grab a chunk of data
            data = self.gps_sock.recv(BUFFER)
            gps_msg = self.parse_gps(data)
            if gps_msg: self.gps_pub.publish(gps_msg)
            rate.sleep()

    def _exit_handler(self):
        '''
        This function runs on module exit.
        '''
        try:
            self.gps_sock.close()
        except:
            pass
        exit()

if __name__ == "__main__":
    reporter = GPS_Reporter()
    reporter.run()
    exit()
    try:
        reporter = GPS_Reporter()
    except:
        rospy.logerr("Failed to start GPS interface.")
    else:
        reporter.run()

