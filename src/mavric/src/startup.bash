#!/bin/bash
source /home/pi/catkin_ws/devel/setup.bash
echo $MAVRIC_BOARD
if [ "$MAVRIC_BOARD" == "Master" ]
then
    roslaunch mavric MasterBoard.launch
elif [ "$MAVRIC_BOARD" == "Drive" ]
then
    # give the master board a little time to turn on
    sleep 15
    roslaunch mavric DriveBoard.launch
fi
