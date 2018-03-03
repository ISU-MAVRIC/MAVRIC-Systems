#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import Adafruit_PCA9685

#STOP = 50
MinTime = 0.001
MaxTime = 0.002
Range   = MaxTime-MinTime
Period  = 0.004
STOP    = MinTime + Range/2

#PWM hat channels
L_CHANNEL = 0
R_CHANNEL = 1

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(1/Period)


#convert from % duty cycle to PWM "ticks" for hat
def to_tick(percent):
        rospy.loginfo(rospy.get_caller_id() + ": %f%%", percent);
        # the 0.92 is a fudge factor that is needed for unokown reasons
	return int(4095 * percent / 0.92)


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	print(data.data)
	#PWM
	
	#if the data string is invalid, kill motors
        data.data = data.data.strip()
	if(len(data.data) != 5):
		rospy.loginfo(rospy.get_caller_id() + " STOPPING")
		pwm.set_pwm(L_CHANNEL, 0, to_tick(STOP/Period))
		pwm.set_pwm(R_CHANNEL, 0, to_tick(STOP/Period))
		return
	
	#get left and right side drive powers
	n1 = Range * (int(data.data[1:3]))/100.0 + MinTime
	n2 = Range * (int(data.data[3:5]))/100.0 + MinTime
	
	#log values and write to PWM channels
	rospy.loginfo(rospy.get_caller_id() + " Left %s, Right %s", n1, n2)
        f1 = to_tick(n1/Period)
        f2 = to_tick(n2/Period)
        rospy.loginfo(rospy.get_caller_id() + ": Left %f, Right %f", f1, f2)
	pwm.set_pwm(L_CHANNEL, 0, to_tick(n1/Period))
	pwm.set_pwm(R_CHANNEL, 0, to_tick(n2/Period))


def listener():
	rospy.init_node('DTS', anonymous=True)
	rospy.Subscriber("/Drive_Train", String, callback)

	rospy.spin()


if __name__ == '__main__':
	listener()







