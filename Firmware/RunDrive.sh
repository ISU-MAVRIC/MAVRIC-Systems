#!/bin/bash

export ROS_MASTER_URI=http://192.168.1.11:11311/

rosparam set /Drive/Left_Back/Channel 0
rosparam set /Drive/Left_Middle/Channel 1
rosparam set /Drive/Left_Front/Channel 2

rosparam set /Drive/Right_Front/Channel 5
rosparam set /Drive/Right_Middle/Channel 4
rosparam set /Drive/Right_Back/Channel 6

rosparam set /Drive/Left_Front/Dir 1
rosparam set /Drive/Left_Middle/Dir 1
rosparam set /Drive/Left_Back/Dir 1

rosparam set /Drive/Right_Front/Dir -1
rosparam set /Drive/Right_Middle/Dir 1
rosparam set /Drive/Right_Back/Dir -1

rosrun mavric Drive_Train_S.py&
