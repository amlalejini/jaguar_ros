#!/usr/bin/env python

import rospy, copy, math
from utils import *
from threading import Lock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32

'''
MODULE TODO LIST:
 - velocity smoothing option? (see aries code)
 - all stop command from controller
 - Controller parameter (logitech, xbox, etc)
'''

#########################################
# Constants
#########################################
# Controller Settings
DEFAULT_LINEAR_AXIS = "LEFT_STICK_VERTICAL"
DEFAULT_ANGULAR_AXIS = "LEFT_STICK_HORIZONTAL"
DEFAULT_HEADLIGHT_BTTN = "Y"
DEFAULT_CONTROLLER_AXIS_SLOP = 0.05
DEFAULT_FRONT_FLIPPER_BTTNS = {"UP":"R1", "DOWN":"RIGHT_TRIGGER"}
DEFAULT_REAR_FLIPPER_BTTNS = {"UP":"L1", "DOWN":"LEFT_TRIGGER"}
# Motor Settings
# DEFAULT_MAX_SPEED:
#   - LINEAR: Max speed in m/s (documented max speed: 5.5km/hr or 1.53m/s)
#   - ANGULAR: Max turning speed in rads/sec (a random guess TODO: make better guess by driving robot in circles)
DEFAULT_MAX_SPEED = {"LINEAR": 1.53, "ANGULAR": math.pi, "FRONT_FLIPPERS": 1, "REAR_FLIPPERS": 1} 
# Topics
DEFAULT_JOY_TOPIC = "joy"
DEFAULT_DRIVE_TOPIC = "cmd_vel"
DEFAULT_FRONT_FLIPPER_CONTROL_TOPIC = "front_flipper_cmds"
DEFAULT_REAR_FLIPPER_CONTROL_TOPIC = "rear_flipper_cmds"
DEFAULT_HEADLIGHT_CONTROL_TOPIC = "headlight_cmds"
#########################################

class Joystick_Controller(object):

    def __init__(self):
        '''
        Joystick Controller contructor
        '''
        rospy.init_node("joystick_controller")

        self.current_joy = None
        self.joy_lock = Lock()
        self.headlight_toggle = False   # False: off, True: on
        self.prev_joy = Joy()  # Stores previous joy message (useful for sequence related commands)

        # Load some controller parameters
        self.linear_axis = rospy.get_param("controller_settings/linear_axis", DEFAULT_LINEAR_AXIS) # Default value
        self.angular_axis = rospy.get_param("controller_settings/angular_axis", DEFAULT_ANGULAR_AXIS) 
        self.headlight_bttn = rospy.get_param("controller_settings/headlight_bttn", DEFAULT_HEADLIGHT_BTTN)
        self.front_flipper_up_bttn = rospy.get_param("controller_settings/front_flipper_up_bttn", DEFAULT_FRONT_FLIPPER_BTTNS["UP"])
        self.front_flipper_down_bttn = rospy.get_param("controller_settings/front_flipper_down_bttn", DEFAULT_FRONT_FLIPPER_BTTNS["DOWN"])
        self.rear_flipper_up_bttn = rospy.get_param("controller_settings/rear_flipper_up_bttn", DEFAULT_REAR_FLIPPER_BTTNS["UP"])
        self.rear_flipper_down_bttn = rospy.get_param("controller_settings/rear_flipper_down_bttn", DEFAULT_REAR_FLIPPER_BTTNS["DOWN"])        

        self.axis_slop_thresh = rospy.get_param("controller_settings/slop", DEFAULT_CONTROLLER_AXIS_SLOP)

        self._check_controller_settings()
        
        # Load some motor settings
        self.max_linear_speed = rospy.get_param("motors/max_speeds/linear_drive", DEFAULT_MAX_SPEED["LINEAR"])
        self.max_angular_speed = rospy.get_param("motors/max_speeds/angular_drive", DEFAULT_MAX_SPEED["ANGULAR"])
        self.max_front_flipper_speed = rospy.get_param("motors/max_speeds/front_flippers", DEFAULT_MAX_SPEED["FRONT_FLIPPERS"])
        self.max_rear_flipper_speed = rospy.get_param("motors/max_speeds/rear_flippers", DEFAULT_MAX_SPEED["REAR_FLIPPERS"])
        # TODO: write function to test the above values

        # Load control topics
        self.drive_topic = rospy.get_param("control_inputs_topics/drive", DEFAULT_DRIVE_TOPIC)
        self.front_flipper_topic = rospy.get_param("control_inputs_topics/front_flipper_control", DEFAULT_FRONT_FLIPPER_CONTROL_TOPIC)
        self.rear_flipper_topic = rospy.get_param("control_inputs_topics/rear_flipper_control", DEFAULT_REAR_FLIPPER_CONTROL_TOPIC)
        self.headlight_topic = rospy.get_param("control_inputs_topics/headlights", DEFAULT_HEADLIGHT_CONTROL_TOPIC)
        self.joy_topic = rospy.get_param("controller_settings/joystick_topic", DEFAULT_JOY_TOPIC)

        # Create necessary ROS publishers
        self.drive_cmds_pub = rospy.Publisher(self.drive_topic, Twist, queue_size = 10)
        self.front_flipper_cmds_pub = rospy.Publisher(self.front_flipper_topic, Float32, queue_size = 10)
        self.rear_flipper_cmds_pub = rospy.Publisher(self.rear_flipper_topic, Float32, queue_size = 10)
        self.headlight_cmds_pub = rospy.Publisher(self.headlight_topic, Bool, queue_size = 10)

        # Subscribe to controller data
        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)

    def _check_controller_settings(self):
        '''
        This function unit tests each of the loaded controller configuration settings.
        If any of the settings fail their unit test, reset the config to default value.
        '''
        #############################
        # Test forward axis 
        #############################
        try:
            LOGITECH_AXES[self.linear_axis]
        except:
            rospy.logerr("Bad FORWARD AXIS configuration.  Resetting to default.")
            self.linear_axis = DEFAULT_LINEAR_AXIS

        try:
            LOGITECH_AXES[self.angular_axis]
        except:
            rospy.logerr("Bad ROTATE AXIS configuration.  Resetting to default.")
            self.rotate_axis = DEFAULT_ANGULAR_AXIS

        try:
            LOGITECH_BUTTONS[self.headlight_bttn]
        except:
            rospy.logerr("Bad HEADLIGHT BTTN configuration.  Resetting to default.")
            self.headlight_bttn = DEFAULT_HEADLIGHT_BTTN

        try:
            self.axis_slop_thresh = float(self.axis_slop_thresh)
        except:
            rospy.logerr("Bad AXIS SLOP THRESHOLD configuration.  Resettings to default.")
            self.axis_slop_thresh = DEFAULT_CONTROLLER_AXIS_SLOP
        # FRONT FLIPPER UP
        try:
            LOGITECH_BUTTONS[self.front_flipper_up_bttn]
        except:
            rospy.logerr("Bad FRONT FLIPPER UP button configuration.  Resetting to default.")
            self.front_flipper_up_bttn = DEFAULT_FRONT_FLIPPER_BTTNS["UP"]
        # FRONT FLIPPER DOWN
        try:
            LOGITECH_BUTTONS[self.front_flipper_down_bttn]
        except:
            rospy.logerr("Bad FRONT FLIPPER DOWN button configuration.  Resetting to default.")
            self.front_flipper_down_bttn = DEFAULT_FRONT_FLIPPER_BTTNS["DOWN"]
        # REAR FLIPPER UP
        try:
            LOGITECH_BUTTONS[self.rear_flipper_up_bttn]
        except:
            rospy.logerr("Bad REAR FLIPPER UP button configuration.  Resetting to default.")
            self.rear_flipper_up_bttn = DEFAULT_REAR_FLIPPER_BTTNS["UP"]
        # REAR FLIPPER DOWN
        try:
            LOGITECH_BUTTONS[self.rear_flipper_down_bttn]
        except:
            rospy.logerr("Bad REAR FLIPPER DOWN button configuration.  Resetting to default.")
            self.rear_flipper_down_bttn = DEFAULT_REAR_FLIPPER_BTTNS["DOWN"]


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
            linear_val = joy.axes[LOGITECH_AXES[self.linear_axis]]      # will be value between -1.0 and 1.0
            angular_val = joy.axes[LOGITECH_AXES[self.angular_axis]]    # will be value between -1.0 and 1.0
            # Calculate velocities
            linear_velocity = 0
            angular_velocity = 0
            # make sure values aren't within axis slop threshold
            if not ((linear_val <= self.axis_slop_thresh) and (linear_val >= -self.axis_slop_thresh)):
                linear_velocity = linear_val * self.max_linear_speed    # m/s
            if not ((angular_val <= self.axis_slop_thresh) and (angular_val >= -self.axis_slop_thresh)):
                angular_velocity = angular_val * self.max_angular_speed # radians/sec
            # Build Twist message
            twister = Twist()
            twister.linear.x = linear_velocity
            twister.angular.z = angular_velocity
            # Publish twist message
            self.drive_cmds_pub.publish(twister)
            ###############################
            # Get front flippers command
            ###############################
            up = True if joy.buttons[LOGITECH_BUTTONS[self.front_flipper_up_bttn]] == 1 else False
            down = True if joy.buttons[LOGITECH_BUTTONS[self.front_flipper_down_bttn]] == 1 else False
            flipper_cmd = Float32()
            if up and not down:
                flipper_cmd.data = self.max_front_flipper_speed
            elif not up and down:
                flipper_cmd.data = -1 * self.max_front_flipper_speed
            else:
                flipper_cmd.data = 0
            self.front_flipper_cmds_pub.publish(flipper_cmd)
            ###############################
            # Get rear flippers command
            ###############################
            up = True if joy.buttons[LOGITECH_BUTTONS[self.rear_flipper_up_bttn]] == 1 else False
            down = True if joy.buttons[LOGITECH_BUTTONS[self.rear_flipper_down_bttn]] == 1 else False
            flipper_cmd = Float32()
            if up and not down:
                flipper_cmd.data = self.max_rear_flipper_speed
            elif not up and down:
                flipper_cmd.data = -1 * self.max_rear_flipper_speed
            else:
                flipper_cmd.data = 0
            self.rear_flipper_cmds_pub.publish(flipper_cmd)
            ###############################
            # Get headlights command
            ###############################
            # toggle headlights on release if last message was a press
            toggle = False if joy.buttons[LOGITECH_BUTTONS[self.headlight_bttn]] == 1 else True
            try:
                # this code will fail on first execution (when the prev joy message is empty)
                prev_input = True if self.prev_joy.buttons[LOGITECH_BUTTONS[self.headlight_bttn]] == 1 else False
            except:
                prev_input = False

            if toggle and prev_input:
                self.headlight_toggle = not self.headlight_toggle
                # publish headlight command (true: on, false: off)
                self.headlight_cmds_pub.publish(Bool(self.headlight_toggle))

            self.prev_joy = joy
            # keep rate
            rate.sleep()

if __name__ == "__main__":
    try:
        jcontroller = Joystick_Controller()
    except:
        rospy.logerr("Failed to start joystick controller.")
    else:
        jcontroller.run()