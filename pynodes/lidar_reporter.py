#!/usr/bin/env python

import rospy, socket, atexit

#########################################
# Constants
#########################################
# Networking
BUFFER = 1024
#########################################

class LIDAR_Reporter(object):

    def __init__(self):
        '''
        LIDAR Reporter contructor.
        '''
        rospy.init_node("lidar_reporter")

        atexit.register(self.exit_handler)

        self.lidar_ip = "192.168.0.70"
        self.lidar_port = 10002

        self.lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.lidar_sock.connect((self.lidar_ip, self.lidar_port))

    def run(self):
        '''
        Main lidar_reporter run function.
        '''
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            data = self.lidar_sock.recv(BUFFER)
            print(str(data))
            rate.sleep()

    def _exit_handler(self):
        '''
        This funtion is called on exit.
        '''
        pass

if __name__ == "__main__":
    reporter = LIDAR_Reporter()
    reporter.run()
