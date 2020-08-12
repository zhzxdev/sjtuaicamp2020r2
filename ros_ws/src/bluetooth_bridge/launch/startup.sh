#!/bin/bash

source /opt/ros/melodic/setup.bash
#export ROS_PACKAGE_PATH=~/ros_ws/src:${ROS_PACKAGE_PATH}
source /home/pi/ros_ws/devel/setup.bash

roslaunch bluetooth_bridge prod.launch
