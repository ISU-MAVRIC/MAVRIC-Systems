#!/usr/bin/env python
import rospy
import Jetson.GPIO as GPIO
from time import time
from std_msgs.msg import Int32

GPIO.setmode(GPIO.BOARD) 
GPIO.setup(31, GPIO.IN) #pin 38
GPIO.setup(33, GPIO.IN) #pin 40

def talker():
	pub = rospy.Publisher('Motor_Feedback', Int32, queue_size=10)
        rospy.init_node('encoderFB', anonymous=True)
	atime = None
	btime = None
	counter = 0
	GPIO.add_event_detect(31, GPIO.RISING)
	GPIO.add_event_detect(33, GPIO.RISING)
        while not rospy.is_shutdown():
            if GPIO.event_detected(31):
                atime = time()
		print('a')
            else:
                atime = None

            if GPIO.event_detected(33):
		print('a')
                btime = time()
            else:
                btime = None

            if atime == None or btime == None:
                counter = counter
            elif atime < btime:
                counter += 1
            elif btime < atime:
                counter -= 1

            pub.publish(counter)
            
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

