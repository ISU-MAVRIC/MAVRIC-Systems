#!/usr/bin/env python
# CAN Robot Enable controller. Implements the robot enable heartbeat required by the Talon SRXs when controlled over CAN. If the Talons do not receive this heartbeat every 100ms, they will shut down.

# Topics:
#   Enabled - Subscription: Listens for input from the user to enable or disable the rover. Defaults to disabled.

import rospy
import os
from std_msgs.msg import Bool
import time

enabled = False

def callback(data):
        global enabled
	enabled = data.data

def listener():
        global enabled

        rospy.init_node('CAN_REn')
        rospy.Subscriber("CAN/TalonsEnabled", Bool, callback, queue_size=10)

        r = rospy.Rate(100) #must send command within 100ms of the last one
        
        while not rospy.is_shutdown():
		if(enabled):
			#command read off the CAN bus using canqv
			#command originates from the Phoenix Tuner app on Windows
			os.system("cansend can0 000401bf#0100");
                        
                r.sleep()
                
if __name__ == '__main__':
    listener()
