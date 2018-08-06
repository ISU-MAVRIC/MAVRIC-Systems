#!/bin/bash
source /home/pi/catkin_ws/devel/setup.bash
echo $MAVRIC_BOARD
if [ "$MAVRIC_BOARD" == "Master" ]
then
    roslaunch mavric MasterBoard.launch
elif [ "$MAVRIC_BOARD" == "Drive" ]
then
    # make sure the master is active before launching
    roslaunch mavric DriveBoard.launch --wait --no-core
elif [ "$MAVRIC_BOARD" == "Arm" ]
then
    roslaunch mavric ArmBoard.launch --wait --no-core
fi
