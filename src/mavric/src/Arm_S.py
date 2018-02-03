#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO




def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)

def listener():
	rospy.init_node('AS', anonymous=True)
	rospy.Subscriber("/Arm", String, callback)
	
	if data.data[0] == 'E':
	
	elif data.data[0] == 'F':

	elif data.data[0] == 'G':

	elif data.data[0] == 'H':

	elif data.data[0] == 'I':

	elif data.data[0] == 'J':

	else:


	rospy.spin()
	GPIO.cleanup()

if __name__ == '__main__':
	listener()









