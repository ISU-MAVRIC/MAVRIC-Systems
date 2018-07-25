#!/bin/bash
source /home/pi/catkin_ws/devel/setup.bash
echo $MAVRIC_BOARD
if [ "$MAVRIC_BOARD" == "Master" ]
then
   roslaunch mavric MasterBoard.launch
fi
