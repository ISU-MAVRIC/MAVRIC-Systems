#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
p = GPIO.PWM(12, 50)
p.start(0)

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	#PWM
	number = float(data.data)
	p.ChangeDutyCycle(number);
	print(number)
	
def listener():
	rospy.init_node('listener0', anonymous=True)
	rospy.Subscriber("/chatter", String, callback)

	rospy.spin()
	GPIO.cleanup()

if __name__ == '__main__':
	listener()
