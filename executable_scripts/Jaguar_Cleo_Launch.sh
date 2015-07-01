#! /bin/bash
cd ~
source .bashrc
source ~/catkin_workspace/devel/setup.bash
roslaunch jaguar_ros jaguar_teleop_cleo.launch
