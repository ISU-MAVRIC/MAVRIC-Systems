#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

#STOP = 50
STOP = 46.9
FORWARD = STOP+20
BACKWARD = STOP-20

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
l = GPIO.PWM(12, 333)
l.start(STOP)

GPIO.setup(33, GPIO.OUT)
r = GPIO.PWM(33, 333)
r.start(STOP)

#scale factor
s=.05

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)
        #PWM

	if data.data == "B":
		l.ChangeDutyCycle(STOP+(50-s*data[1:2]));
		r.ChangeDutyCycle(STOP+(50-s*data[1:2]));
	elif data.data == "R":
		r.ChangeDutyCycle(STOP+s*data[1:2]);
	elif data.data == "L":
		l.ChangeDutyCycle(STOP+s*data[1:2]);
	elif data.data == "D":
		l.ChangeDutyCycle(STOP+(50-s*data[1:2]));
		r.ChangeDutyCycle(STOP+(50-s*data[3:4]));
	else:
		l.ChangeDutyCycle(STOP);
		r.ChangeDutyCycle(STOP);

def listener():
	rospy.init_node('DTS', anonymous=True)
	rospy.Subscriber("/Drive_Train", String, callback)

	rospy.spin()
	GPIO.cleanup()

if __name__ == '__main__':
	listener()









