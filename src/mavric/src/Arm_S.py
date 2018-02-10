#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO




def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)

def arm1():
	rospy.init_node('AS1', anonymous=True)
	rospy.Subscriber("/arm1", String, callback)

	rospy.spin()
#	GPIO.cleanup()

while __name__ == '__main__':
	arm1()









