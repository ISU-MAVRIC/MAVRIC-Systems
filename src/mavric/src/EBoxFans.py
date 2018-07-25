#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import Adafruit_PCA9685
import time

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(25000)

def callback(data):
        rospy.loginfo(rospy.get_caller_id() + " Setting fans to %6.2f%%", data.data*100)
        pwm.set_pwm(0, 0, int(data.data*4095+0.5))
        pwm.set_pwm(1, 0, int(data.data*4095+0.5))

def listener():

        rospy.init_node('EBoxFans')
        rospy.Subscriber("/Fan_Setpoint", Float32, callback, queue_size=10)
        rospy.spin()



if __name__ == '__main__':
    listener()
