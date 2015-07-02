# jaguar_ros
This project is a rewrite/refactoring of the Dr. Robot Jaguar's ROS code.  The jaguar_player is a refactoring of the drrobot_player node, and the motion sensor driver files are a refactoring of the original drrobot motion sensor driver files.

## Important information when using Logitech controller.
* Make sure that the switch on the back of the controller (between the shoulder buttons) is flipped to the 'D' position.  Otherwise, the trigger buttons will be axes instead of buttons.  The joystick controller node expects them as buttons.  If you want the triggers to be axes, some tweaking to the joystick controller, controller config file, and button mappings may be necessary.

## Dependencies
* [Axis Camera ROS Driver](http://wiki.ros.org/axis_camera)
* [camera_info_manager_py](https://github.com/ros-perception/camera_info_manager_py)
* [Joystick Drivers for ROS](https://github.com/ros-drivers/joystick_drivers)
