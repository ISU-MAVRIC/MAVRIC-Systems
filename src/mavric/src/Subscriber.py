#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
l = GPIO.PWM(12, 50)
l.start(7.08)

GPIO.setup(33, GPIO.OUT)
r = GPIO.PWM(33, 50)
r.start(7.08)

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)
	#PWM
	if data.data == "f":
		l.ChangeDutyCycle(9);
		r.ChangeDutyCycle(9);
	elif data.data == "b":
		l.ChangeDutyCycle(6);
		r.ChangeDutyCycle(6);
	elif data.data == "r":
		l.ChangeDutyCycle(9);
		r.ChangeDutyCycle(6);
	elif data.data == "l":
		l.ChangeDutyCycle(6);
		r.ChangeDutyCycle(9);
	else:
		l.ChangeDutyCycle(7.08);
		r.ChangeDutyCycle(7.08);
	
def listener():
	rospy.init_node('listener0', anonymous=True)
	rospy.Subscriber("/chatter", String, callback)

	rospy.spin()
	GPIO.cleanup()

if __name__ == '__main__':
	listener()
