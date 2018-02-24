#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO




def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)	
	if data.data[0] == 'E':

	elif data.data[0] == 'F':

	elif data.data[0] == 'G':

	elif data.data[0] == 'H':

	elif data.data[0] == 'I':

	elif data.data[0] == 'J':

	else:
	#actuate the arm.

def arm1():
	while not rospy.is_shutdown()
		rospy.init_node('AS1', anonymous=True)
		rospy.Subscriber("/arm1", String, callback)

		rospy.spin()

	GPIO.cleanup()

if __name__ == '__main__':
	arm1()









