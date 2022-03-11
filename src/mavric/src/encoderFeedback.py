#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from time import time
from std_msgs.msg import Float64
from threading import Thread

GPIO.setmode(GPIO.BOARD) 

count = 0

def detect():
	global count
	if GPIO.event_detected(38):
			if GPIO.input(40) == 0:
				count += 1
			else:
				count -=1

def talker():
	pub = rospy.Publisher('Motor_Feedback', Float64, queue_size=10)
	rospy.init_node('encoderFB')
	pin1 = rospy.get_param("ch1", 38)
	pin2 = rospy.get_param("ch2", 40)
	global count
	GPIO.add_event_detect(pin1, GPIO.RISING)
	count = Thread(target=detect)
	count.start()
	while not rospy.is_shutdown():
		pub.publish(count)
		    
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		count.join()


