#!/usr/bin/env python

import rospy, socket, atexit


'''
This module is hopefully just a TEMPORARY development module.
This module will communicate with the mysterious robotboard on the jaguar.
'''

#########################################
# Constants
#########################################
# Networking
DEFAULT_JAGUAR_IP = "192.168.0.70"
DEFAULT_JAGUAR_PORT = 10001
BUFFER = 2056
# Topic(s)
#########################################

class Jaguar_Interface(object):

    def __init__(self):
        '''
        Jaguar robot interface constructor.
           - Used to explore all the things drrobot_player interfaces with.
        '''

        rospy.init_node("jaguar_interface")

        atexit.register(self._exit_handler)

        # Setup networking
        self.jaguar_ip = DEFAULT_JAGUAR_IP
        self.jaguar_port = DEFAULT_JAGUAR_PORT
        # Create jaguar comms socket.
        self.jag_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Attempt to connect
        rospy.loginfo("Trying to connect to mystery board at " + str(self.jaguar_ip) + " on port " + str(self.jaguar_port))
        while not rospy.is_shutdown():
            try:
                self.jag_sock.connect((self.jaguar_ip, self.jaguar_port))
            except:
                rospy.logerr("Failed to connect to Jaguar mystery board. Will continue trying.")
                rospy.sleep(3)
            else:
                rospy.loginfo("Successfully connected to Jaguar mystery board.")
                break

    def run(self):
        '''
        Main loop run function.
        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Grab chunk of data
            print("==================")
            print("Waiting for new data...")
            data = self.jag_sock.recv(BUFFER)
            print("DATA RECEIVED: ")
            print(str(data))
            rate.sleep()



    def _exit_handler(self):
        '''
        This function gets called on exit.
            - Cleans up socket connection.
        '''
        try:
            self.jag_sock.close()
        except:
            pass
        exit()

if __name__ == "__main__":
    try:
        interface = Jaguar_Interface()
    except:
        rospy.logerr("Failed to start jag interface.")
    else:
        interface.run()