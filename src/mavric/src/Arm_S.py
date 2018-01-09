#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO




def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)
	# actuate the arm.

def listener():
	rospy.init_node('AS', anonymous=True)
	rospy.Subscriber("/Arm", String, callback)

	rospy.spin()
	GPIO.cleanup()

if __name__ == '__main__':
	listener()









