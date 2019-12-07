#!/bin/bash
chmod 666 /dev/ttyS*
source /home/mavric/catkin_ws/devel/setup.bash
roslaunch mavric jetson.launch
