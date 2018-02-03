#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO




def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)

def listener():
	rospy.init_node('SS', anonymous=True)
	rospy.Subscriber("/Science", String, callback)

	if data.data[0] == 'M':

	elif data.data[0] == 'N':

	elif data.data[0] == 'O':

	elif data.data[0] == 'P':

	else:

	rospy.spin()
	GPIO.cleanup()

if __name__ == '__main__':
	listener()









