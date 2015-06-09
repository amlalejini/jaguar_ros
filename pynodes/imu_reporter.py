#!/usr/bin/env python

import rospy, socket, atexit, re
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion 

'''
MODULE TODO LIST:
 - change defaults to online docs
 - make robust
 - have Ryan look at calculating heading
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
        # magnetometer updates at a different rate than other sensors on IMU, need to keep around most recent values
        self.current_mag = [0, 0, 0]

        # Register an exit handler.
        atexit.register(self._exit_handler)
        
        # Setup networking with imu
        # Load imu ip and port from parameter server.
        self.imu_ip = rospy.get_param("sensors/imu/ip", DEFAULT_IMU_IP)
        self.imu_port = rospy.get_param("sensors/imu/port", DEFAULT_IMU_PORT)
        # Make sure port is okay
        try:
            int(self.imu_port)
        except:
            rospy.logerr("Bad IMU port given. Resetting to default.")
            self.imu_port = DEFAULT_IMU_PORT
        # Create IMU comms socket (TCP communication protocol)
        self.imu_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Attempt to connect
        rospy.loginfo("Trying to connect to IMU at " + str(self.imu_ip) + " on port " + str(self.imu_port))
        while not rospy.is_shutdown():
            try:
                self.imu_sock.connect((self.imu_ip, self.imu_port))
            except:
                rospy.logerr("Failed to connect to IMU.  Will continue trying.")
                rospy.sleep(3)
            else:
                rospy.loginfo("Successfully connected to IMU.")

        # Load topic name(s)
        self.imu_topic = rospy.get_param("sensors/imu/topic", DEFAULT_IMU_TOPIC)
        # Create necessary ROS publishers
        self.imu_pub = rospy.Publisher(self.imu_topic, Imu)#, queue_size = 10)

    def run(self):
        '''
        Main loop run function.
        '''
        rate = rospy.Rate(75)  # The magnetometer data gets updated at ~50hz, other sensors on IMU update much faster.

        while not rospy.is_shutdown():
            # Grab a chunk of data
            data = self.imu_sock.recv(BUFFER)

            # Use regular expressions to extract complete message
            # message format: $val,val,val,val,val,val,val,val,val,val#\n
            hit = re.search(r"\$-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*#", data)
            if hit != None:
                imu_data = hit.group(0)
                try:
                    # Try to format the data
                    imu_data = imu_data[1:-1].split(",")
                    #Format (from drrobot docs): seq, accelx, accely, accelz, gyroY, gyroZ, gyroX, magnetonX, magnetonY, magnetonZ
                    seq = int(imu_data[0])

                    accelx = float(imu_data[1])
                    accely = float(imu_data[2])
                    accelz = float(imu_data[3])

                    gyroy = float(imu_data[4])
                    gyroz = float(imu_data[5])
                    gyrox = float(imu_data[6])

                    magnetonx = float(imu_data[7])
                    magnetony = float(imu_data[8])
                    magnetonz = float(imu_data[9])
                except:
                    # bad data in match, pass
                    pass
                else:
                    # data formatted fine, build message and publish

                    # if we didn't get a magnetometer update, set to current reading
                    if magnetonz == 0:
                        magnetonx = self.current_mag[0]
                        magnetony = self.current_mag[1]
                        magnetonz = self.current_mag[2]
                    # otherwise, update current magnetometer
                    else:
                        self.current_mag = [magnetonx, magnetony, magnetonz]


                    # Build message 
                    # can use current_mag reading here
                    msg = Imu()
                    msg.header = Header(stamp = rospy.Time.now())
                    msg.linear_acceleration = Vector3(accelx, accely, accelz)
                    msg.angular_velocity = Vector3(gyrox, gyroy, gyroz)
                    msg.orientation = Quaternion()
                    # Publish message
                    self.imu_pub.publish(msg)

            # publish
            rate.sleep()
            
    def _exit_handler(self):
        '''
        This function runs on exit.
        '''
        try:
            self.imu_sock.close()
        except:
            pass
        finally:
            exit()

if __name__ == "__main__":
    reporter = IMU_Reporter()
    reporter.run()
