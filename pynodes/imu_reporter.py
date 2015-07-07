#!/usr/bin/env python

import rospy, socket, atexit, re
from math import sqrt
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion 

'''
MODULE TODO LIST:
 - change defaults to online docs
 - have Ryan look at calculating heading
'''

__authors__ = ["Alex Lalejini", "Ryan Smith"]

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
        
        # self.mag_fp = open("mag_cal.txt", "a")
        # self.gyro_fp = open("gyro_cal_up.txt", "a")
        # self.accel_fp = open("accel_cal_up.txt", "a")

        # Register an exit handler.
        atexit.register(self._exit_handler)
        
        # Setup networking with imu
        # Load imu ip and port from parameter server.
        self.imu_ip = rospy.get_param("sensors/imu/ip", DEFAULT_IMU_IP)
        self.imu_port = rospy.get_param("sensors/imu/port", DEFAULT_IMU_PORT)
        # Create IMU comms socket (TCP communication protocol)
        self.imu_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.imu_sock.settimeout(2.0)
        # Attempt to connect
        self.connect()

        # Load topic name(s)
        self.imu_topic = rospy.get_param("sensors/imu/topic", DEFAULT_IMU_TOPIC)
        # Create necessary ROS publishers
        self.imu_pub = rospy.Publisher(self.imu_topic, Imu)#, queue_size = 10)
    
    def connect(self):
        '''
        This function attempts to connect to the IMU 
        '''
        rospy.loginfo("Trying to connect to IMU at " + str(self.imu_ip) + " on port " + str(self.imu_port))
        while not rospy.is_shutdown():
            try:
                self.imu_sock.connect((self.imu_ip, self.imu_port))
            except:
                rospy.logerr("Failed to connect to IMU.  Will continue trying.")
                rospy.sleep(3)
            else:
                rospy.loginfo("Successfully connected to IMU.")
                break


    def sign_of(self, num):
        '''
        Given number, return 1 if number is positive, -1 if negative.
        '''
        return 1.0 if num >= 0.0 else -1.0

    def rot_mat_to_quat(self, rot_mat):
        '''
        Given a 3x3 rotation matrix, convert to quaternion.
        rotation matrix form: [ [r11, r12, r13]
                                [r21, r22, r23]
                                [r31, r32, r33] ]
        NOTE: This conversion function comes from: http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche52.html
        '''

        try:
            r11 = rot_mat[0][0]
            r12 = rot_mat[0][1]
            r13 = rot_mat[0][2]
            r21 = rot_mat[1][0]
            r22 = rot_mat[1][1]
            r23 = rot_mat[1][2]
            r31 = rot_mat[2][0]
            r32 = rot_mat[2][1]
            r33 = rot_mat[2][2]
        except:
            return None

        q0 = (r11 + r22 + r33 + 1.0) / 4.0
        q1 = ( r11 - r22 - r33 + 1.0) / 4.0;
        q2 = (-r11 + r22 - r33 + 1.0) / 4.0;
        q3 = (-r11 - r22 + r33 + 1.0) / 4.0;
        if (q0 < 0.0): q0 = 0.0
        if (q1 < 0.0): q1 = 0.0
        if (q2 < 0.0): q2 = 0.0
        if (q3 < 0.0): q3 = 0.0
        q0 = sqrt(q0)
        q1 = sqrt(q1)
        q2 = sqrt(q2)
        q3 = sqrt(q3)
        if (q0 >= q1 and q0 >= q2 and q0 >= q3):
            q0 *= +1.0;
            q1 *= self.sign_of(r32 - r23);
            q2 *= self.sign_of(r13 - r31);
            q3 *= self.sign_of(r21 - r12);
        elif (q1 >= q0 and q1 >= q2 and q1 >= q3):
            q0 *= self.sign_of(r32 - r23);
            q1 *= +1.0;
            q2 *= self.sign_of(r21 + r12);
            q3 *= self.sign_of(r13 + r31);
        elif (q2 >= q0 and q2 >= q1 and q2 >= q3):
            q0 *= self.sign_of(r13 - r31);
            q1 *= self.sign_of(r21 + r12);
            q2 *= +1.0;
            q3 *= self.sign_of(r32 + r23);
        elif (q3 >= q0 and q3 >= q1 and q3 >= q2):
            q0 *= self.sign_of(r21 - r12);
            q1 *= self.sign_of(r31 + r13);
            q2 *= self.sign_of(r32 + r23);
            q3 *= +1.0;
        else:
            return None
        r = sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        return [q0 / r, q1 / r, q2 / r, q3 / r]



    def parse_imu(self, data):
        '''
        Given data from jaguar imu, parse and return a standard IMU message.
        Return None when given bad data, or no complete message was found in data.
        '''
         # Use regular expressions to extract complete message
        # message format: $val,val,val,val,val,val,val,val,val,val#\n
        hit = re.search(r"\$-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*,-?[0-9]*#", data)
        if not hit:
            # if there are no hits, return None
            return None
        else:
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

                rot_mat = [ [float(imu_data[10]), float(imu_data[11]), float(imu_data[12])],
                            [float(imu_data[13]), float(imu_data[14]), float(imu_data[15])],
                            [float(imu_data[16]), float(imu_data[17]), float(imu_data[18])] ]

            except:
                # bad data in match, pass
                return None
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

                # Calculate quaternion from given rotation matrix
                quat = rot_mat_to_quat(rot_mat);
                # Build message 
                msg = Imu()
                msg.header = Header(stamp = rospy.Time.now(), frame_id = "imu")
                msg.linear_acceleration = Vector3(accelx, accely, accelz)
                msg.angular_velocity = Vector3(gyrox, gyroy, gyroz)
                if quat != None: msg.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                return msg

    def run(self):
        '''
        Main loop run function.
        '''
        rate = rospy.Rate(75)  # The magnetometer data gets updated at ~50hz, other sensors on IMU update much faster.
        while not rospy.is_shutdown():
            try:
                # Grab a chunk of data
                data = self.imu_sock.recv(BUFFER)
            except socket.timeout:
                # socket timed out
                # close socket
                self.imu_sock.close()
                # create new imu socket
                self.imu_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.imu_sock.settimeout(2.0)
                rospy.loginfo("IMU Socket timed out.  Attempting to reconnect.")
                # reconnect
                self.connect()
            else:
                # Publish message
                imu_msg = self.parse_imu(data)
                if imu_msg: self.imu_pub.publish(imu_msg)

            rate.sleep()
            
    def _exit_handler(self):
        '''
        This function runs on exit.
        '''
        # try:
        #     self.mag_fp.close()
        #     self.accel_fp.close()
        #     self.gyro_fp.close()
        # except:
        #     pass
        try:
            self.imu_sock.close()
        except:
            pass
        finally:
            exit()
            

if __name__ == "__main__":
    try:
        reporter = IMU_Reporter()
    except:
        rospy.logerr("Failed to start IMU interface.")
    else:
        reporter.run()
