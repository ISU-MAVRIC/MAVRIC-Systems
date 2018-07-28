#!/usr/bin/env python

# Subscribes to a topic for each PWM output, setting the output accordingly.
# Each topic is a std_msgs/Float64 topic named CH# where # is the channel number (0-15)

import rospy
from std_msgs.msg import Float64
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(25000)

def callback(data, channel):
        rospy.loginfo(rospy.get_caller_id() + " Setting channel " + str(channel) +"  to %6.2f%%", data.data*100)
        pwm.set_pwm(channel, 0, int(data.data*4095+0.5))

def listener():
        rospy.init_node('PCA9685_PWM_HAT')
        freq = rospy.get_param('~frequency', 50);
        pwm.set_pwm_freq(freq)
        for i in range(16):
                rospy.Subscriber("PWM_Channels/CH"+str(i), Float64, callback, i, queue_size=10)

        rospy.spin()



if __name__ == '__main__':
    listener()
