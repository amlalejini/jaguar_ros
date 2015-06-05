#!/usr/bin/env python

import rospy, copy
from utils import *
from threading import Lock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class Joystick_Controller(object):

    def __init__(self):
        '''
        Joystick Controller contructor
        '''
        rospy.init_node("joystick_controller")

        self.current_joy = None
        self.joy_lock = Lock()

        # Load some controller parameters, TODO: TEST MAKE SURE EXIT
        self.forward_axis = rospy.get_param("", "LEFT_STICK_VERTICAL") # Default value
        self.rotate_axis = rospy.get_param("", "LEFT_STICK_HORIZONTAL") 
        try:
            # do stuff
        except:
            # stick with defaults
    

        self.drive_topic = ""
        self.front_flipper_topic = ""
        self.rear_flipper_topic = ""
        self.headlight_topic = ""
        self.joy_topic = ""

        self.drive_cmds_pub = rospy.Publisher(self.drive_topic, Twist)
        self.front_flipper_cmds_pub = rospy.Publisher(self.front_flipper_topic, None)
        self.rear_flipper_cmds_pub = rospy.Publisher(self.rear_flipper_topic, None)
        self.headlight_cmds_pub = rospy.Publisher(self.headlight_topic, None)

        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)

    def joy_callback(self, data):
        '''
        Joy stick callback
        '''
        # safely update current joy
        with self.joy_lock:
            self.current_joy = copy.deepcopy(data)

    def run(self):
        '''
        Main run loop
        '''
        # wait for first joystick message
        rospy.wait_for_message(self.joy_topic, Joy)
        # create target rate to attempt to keep run loop at (10hz is good)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # safely grab most recent joy message
            joy = None
            with self.joy_lock:
                joy = self.current_joy

            ###############################
            # Get drive command
            ###############################
            linear_val = joy.axes[LOGITECH_AXES[self.forward_axis]]
            ###############################
            # Get front flippers command
            ###############################

            ###############################
            # Get rear flippers command
            ###############################

            ###############################
            # Get headlights command
            ###############################

            # keep rate
            rate.sleep()

if __name__ == "__main__":
    pass