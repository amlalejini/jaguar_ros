#!/usr/bin/env python

import rospy
from threading import Lock
from std_msgs.msg import Bool

'''
This module is responsible for controlling Jaguar headlights via ROS commands.

MODULE TODO LIST:
 - add in comms with jaguar board
'''

#########################################
# Constants
#########################################
# Topics
DEFAULT_HEADLIGHT_CONTROL_TOPIC = "headlight_cmds"
#########################################

class Headlight_Controller(object):

    def __init__(self):
        '''
        Headlight controller constructor.
        '''
        rospy.init_node("headlight_controller")

        self.current_cmd = None
        self.received_cmd = None
        self.cmd_lock = Lock()

        # Load topic name(s)
        self.headlight_topic = rospy.get_param("robot_control_topics/headlight_control", DEFAULT_HEADLIGHT_CONTROL_TOPIC)
        # Setup subscriptions
        rospy.Subscriber(self.headlight_topic, Bool, self.headlight_cmds_callback)

    def headlight_cmds_callback(self, data):
        '''
        This function is called when a message is transmitted over headlight topic.
        '''
        with self.cmd_lock:
            self.received_cmd = True
            self.current_cmd = data.data


    def run(self):
        '''
        Main run loop.  Use most recent headlight command to send commands to Jaguar.
        '''
        # wait for first headlight command before running
        rospy.wait_for_message(self.headlight_topic, Bool)
        # create target rate to attempt to keep run loop running at (10hz is fine)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Safely grab most recent headlight cmd
            cmd = None
            cmd_recvd = None
            with self.cmd_lock:
                cmd_recvd = self.received_cmd
                if cmd_recvd: self.received_cmd = False
                cmd = self.current_cmd

            if cmd_recvd:
                # True: turn headlights on
                # False: turn headlights off
                if cmd:
                    # turn headlights on 
                    print("turning headlights on...")
                    pass # TODO: write this code
                else:
                    # turn headlights off
                    print("turning headlights off...")
                    pass # TODO: write this code

            rate.sleep()

        

if __name__ == "__main__":
    try:
        controller = Headlight_Controller()
    except:
        rospy.logerr("Failed to start headlight controller.")
    else:
        controller.run()