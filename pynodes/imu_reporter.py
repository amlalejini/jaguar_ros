#!/usr/bin/env python

import rospy, socket, atexit, re, time
from sensor_msgs.msg import Imu

'''
MODULE TODO LIST:
 - change defaults to online docs
 - make robust
'''

#########################################
# Constants
#########################################
# Networking
DEFAULT_IMU_IP = "192.168.0.71"
DEFAULT_IMU_PORT = 10001
BUFFER = 64
# Topic(s)
DEFAULT_IMU_TOPIC = "jaguar_imu"
#########################################

class IMU_Reporter(object):

    def __init__(self):
        '''
        GPS Reporter contructor.
        '''
        rospy.init_node("imu_reporter")
        atexit.register(self._exit_handler)
        # Setup networking with imu
        self.imu_ip = rospy.get_param("sensors/imu/ip", DEFAULT_IMU_IP)
        self.imu_port = rospy.get_param("sensors/imu/port", DEFAULT_IMU_PORT)
        self.imu_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.imu_sock.connect((self.imu_ip, self.imu_port))
        # Load topic name(s)
        self.imu_topic = rospy.get_param("sensors/imu/topic", DEFAULT_IMU_TOPIC)
        # Create necessary ROS publishers
        self.imu_pub = rospy.Publisher(self.imu_topic, Imu)#, queue_size = 10)

    def run(self):
        '''
        Main loop run function.
        '''
        rate = rospy.Rate(75)
        recv_thing = time.time()
        while not rospy.is_shutdown():

            data = self.imu_sock.recv(BUFFER)
            #print("==========")
            # Use regular expressions to extract complete message
            # message format: $val,val,val,val,val,val,val,val,val,val#\n
            
            m = re.search(r"\$-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*#", data)
            if m != None:
                imu_data = m.group(0)
                imu_data = imu_data[1:-1].split(",") 
                # seq, accelx, accely, accelz, gyroY, gyroZ, gyroX, magnetonX, magnetonY, magnetonZ  
                # only update mag data if magz is not 0       
                if imu_data[-1] != "0": 
                    print("====")
                    print(imu_data)
                    print("TIME: " + str(time.time() - recv_thing))
                    recv_thing = time.time()
            else:
                print("NO MATCH")
            # publish
            rate.sleep()
            
    def _exit_handler(self):
        '''
        This function runs on exit.
        '''
        self.imu_sock.close()

if __name__ == "__main__":
    reporter = IMU_Reporter()
    reporter.run()
