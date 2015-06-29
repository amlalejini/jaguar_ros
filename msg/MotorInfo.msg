# motor sensor data message from DrRobot Robot.

Header header    	    # timestamp in the header is the time the driver
		 	            # returned the battery/power reading
string robot_type	    # robot type, I90 series, sentinel3 or Jaguar Power/Motion

uint32 encoder_pos	    # encoder positon count
uint32 encoder_vel	    # encoder velocity value; rotation speed (unit is defined as pulse change within 1 second.  And it is the absolute value)
uint32 encoder_dir	    # encoder direction; -1, 0, 1.  1: positive dir; 0: no movement; -1: negative dir

float32 motor_current	# motor current
uint32 motor_pwm	    # output PWM value, only for Jaguar series robot
