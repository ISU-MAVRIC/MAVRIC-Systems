#!/usr/bin/env python
# CAN Robot Enable controller. Implements the robot enable heartbeat required by the Talon SRXs when controlled over CAN. If the Talons do not receive this heartbeat every 100ms, they will shut down.

# Topics:
#   Enabled - Subscription: Listens for input from the user to enable or disable the rover. Defaults to disabled.

import rospy
import os
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import time

enabled = False
val = 0

def en_callback(data):
        global enabled
	enabled = data.data

def val_callback(data):
	global val
	val = data.data

def listener():
        global enabled
	global val

        rospy.init_node('CAN_RTst')
        rospy.Subscriber("CAN/TalonsEnabled", Bool, en_callback, queue_size=10)
	rospy.Subscriber("CAN/TestValue", Float64, val_callback, queue_size=10)

        r = rospy.Rate(100) #must send command within 10ms of the last one (100Hz)
        
        while not rospy.is_shutdown():
		if(enabled):
			i_clamp = max(min(1023, int(val)), -1023)
			os.system("cansend can0 02040207#{:06x}0000001000".format(i_clamp & 0xffffff, 'x'))
                        #os.system("cansend can0 02040207#0003ff0000001000")
                r.sleep()
                
if __name__ == '__main__':
    listener()
