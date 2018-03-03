#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import Adafruit_PCA9685

#STOP = 50
STOP = 47.5
FORWARD = STOP + 20
BACKWARD = STOP - 20

#PWM hat channels
L_CHANNEL = 0
R_CHANNEL = 1

#scale factor 
SF = -0.5

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(333)


#convert from % duty cycle to PWM "ticks" for hat
def to_tick(percent):
	return int(4095 * (percent / 100))


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)
	#PWM
	
	#if the data string is invalid, kill motors
	if(len(data.data) != 5):
		rospy.loginfo(rospy.get_caller_id() + " STOPPING")
		pwm.set_pwm(L_CHANNEL, 0, to_tick(STOP))
		pwm.set_pwm(R_CHANNEL, 0, to_tick(STOP))
		return
	
	#get left and right side drive powers
	n1 = STOP + (SF * (int(data.data[1:3]) - 50))
	n2 = STOP + (SF * (int(data.data[3:5]) - 50))
	
	#log values and write to PWM channels
	rospy.loginfo(rospy.get_caller_id() + " Left %s, Right %s", n1, n2)
	pwm.set_pwm(L_CHANNEL, 0, to_tick(n1))
	pwm.set_pwm(R_CHANNEL, 0, to_tick(n2))


def listener():
	rospy.init_node('DTS', anonymous=True)
	rospy.Subscriber("/Drive_Train", String, callback)

	rospy.spin()


if __name__ == '__main__':
	listener()







