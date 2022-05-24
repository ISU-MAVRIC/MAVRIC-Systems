#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from time import time
from std_msgs.msg import Float64
from threading import Thread

GPIO.setmode(GPIO.BOARD) 

count = 0.0
direction = 1
pin1 = 0
pin2 = 0

def detect():
	global count, direction
	count += 1*direction

def setForward():
	global direction
	direction = 1

def setBackward():
	global direction
	direction = -1

def talker():
	global pin1, pin2
	pub = rospy.Publisher('Motor_Feedback', Float64, queue_size=10)
	rospy.init_node('encoderFB')
	pin1 = rospy.get_param("~ch1", 38)
	pin2 = rospy.get_param("~ch2", 40)
	GPIO.setup(pin1, GPIO.IN)
	GPIO.setup(pin2, GPIO.IN)
	global count
	GPIO.add_event_detect(pin1, GPIO.RISING, callback=detect)
	GPIO.add_event_detect(pin2, GPIO.BOTH, callback=setBackward)
	GPIO.add_event_detect(pin2, GPIO.FALLING, callback=setForward)
	while not rospy.is_shutdown():
		pub.publish(count)
		    
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		GPIO.cleanup()

