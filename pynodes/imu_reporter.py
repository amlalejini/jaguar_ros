#!/usr/bin/env python

import rospy, socket

'''
MODULE TODO LIST:
 - change defaults to online docs
 - make robust
'''

#########################################
# Constants
#########################################
# Networking
DEFAULT_IMU_IP = 192.168.0.71
DEFAULT_IMU_PORT = 10001
BUFFER = 128
# Topic(s)
DEFAULT_IMU_TOPIC = "jaguar_imu"
#########################################

class IMU_Reporter(object):

    def __init__(self):
        '''
        GPS Reporter contructor.
        '''
        rospy.init_node("imu_reporter")

        # Setup networking with imu
        self.imu_ip = rospy.get_param("sensors/imu/ip", DEFAULT_IMU_IP)
        self.imu_port = rospy.get_param("sensors/imu/port", DEFAULT_IMU_PORT)
        self.imu_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.imu_sock.connect((self.imu_ip, self.imu_port))
        # Load topic name(s)
        self.imu_topic = rospy.get_param("sensors/imu/topic", DEFAULT_IMU_TOPIC)
        # Create necessary ROS publishers
        self.imu_pub = rospy.Publisher(self.imu_topic, None, queue_size = 10)

    def run(self):
        '''
        Main loop run function.
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            data = self.imu_sock.recv(BUFFER)
            print(data)
            # do some regex
            # publish
            rate.sleep()

if __name__ == "__main__":
    reporter = IMU_Reporter()
    retporter.run()