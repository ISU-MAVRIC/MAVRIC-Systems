#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from time import time
from std_msgs.msg import Int32

GPIO.setmode(GPIO.BOARD) 
GPIO.setup(38, GPIO.IN) #pin 38
GPIO.setup(40, GPIO.IN) #pin 40

def talker():
	pub = rospy.Publisher('Motor_Feedback', Int32, queue_size=10)
	rospy.init_node('encoderFB')
	atime = None
	btime = None
	counter = 0
	high = 0
	GPIO.add_event_detect(38, GPIO.RISING)
	GPIO.add_event_detect(40, GPIO.RISING)
	while not rospy.is_shutdown():
		if GPIO.event_detected(38):
			if GPIO.input(40) == 0:
				counter += 1
			else:
				counter -=1

		pub.publish(counter)
		    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

