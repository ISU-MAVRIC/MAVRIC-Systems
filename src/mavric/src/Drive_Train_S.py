#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

#STOP = 50
STOP = 47.5
FORWARD = STOP+20
BACKWARD = STOP-20

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)
l = GPIO.PWM(32, 333)
l.start(STOP)

GPIO.setup(33, GPIO.OUT)
r = GPIO.PWM(33, 333)
r.start(STOP)

#scale factor 
s=-.5

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)
        #PWM

	if data.data[1] == 'B':
		rospy.loginfo(rospy.get_caller_id() + "Left %s, Right %s", STOP+(s*(int(data.data[2:4])-50)), STOP+(s*(int(data.data[2:4])-50)));
		l.ChangeDutyCycle(STOP+(s*(int(data.data[2:4])-50)));
		r.ChangeDutyCycle(STOP+(s*(int(data.data[2:4])-50)));
	elif data.data[1] == 'R':
		rospy.loginfo(rospy.get_caller_id() + "Right %s", STOP+s*(int(data.data[2:4])-50));
		r.ChangeDutyCycle(STOP+s*(int(data.data[2:4])-50));
	elif data.data[1] == 'L':
		rospy.loginfo(rospy.get_caller_id() + "Left %s", STOP+s*(int(data.data[2:4])-50));
		l.ChangeDutyCycle(STOP+s*(int(data.data[2:4])-50));
	elif data.data[1] == 'D':
		rospy.loginfo(rospy.get_caller_id() + "Left %s, Right %s", STOP+(s*(int(data.data[2:4])-50)), STOP+(s*(int(data.data[3:4])-50)));
		l.ChangeDutyCycle(STOP+(s*data.data[2:4])-50);
		r.ChangeDutyCycle(STOP+(s*data.data[4:6])-50);
	else:
		rospy.loginfo(rospy.get_caller_id() + "STOPPING");
		l.ChangeDutyCycle(STOP);
		r.ChangeDutyCycle(STOP);

def listener():
	rospy.init_node('DTS', anonymous=True)
	rospy.Subscriber("/Drive_Train", String, callback)

	rospy.spin()
	GPIO.cleanup()

if __name__ == '__main__':
	listener()









