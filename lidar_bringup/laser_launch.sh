#!/bin/sh
sudo chmod a=r+w /dev/pts/*
sudo chmod a=r+w /dev/ttyS51
rosparam set hokuyo_node/port /dev/ttyS51
rosrun hokuyo_node hokuyo_node
