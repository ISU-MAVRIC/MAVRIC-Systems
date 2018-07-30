#!/usr/bin/env python

# Subscribes to a topic for each PWM output, setting the output accordingly.
# Each topic is a std_msgs/Float64 topic named CH# where # is the channel number (0-15)

# Parameters:
#  ~frequency - the frequency in Hz of the desired PWM
#  ~clk_error - A calibration parameter for the PWM's clock rate error
#                 A number greater than 1 means that the clock runs faster than it should

import rospy
from std_msgs.msg import Float64
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
period = 0

def time_callback(data, channel):
        time = data.data
        percent = time/period;
        pwm.set_pwm(channel, 0, int(percent*4095+0.5))

def percent_callback(data, channel):
        percent = data.data
        pwm.set_pwm(channel, 0, int(percent*4095+0.5))

def listener():
        global period
        rospy.init_node('PCA9685_PWM_HAT')
        freq = rospy.get_param('~frequency', 50);
        period = 1./freq
        clk_error = rospy.get_param('~clk_error', 1);
        print(freq, period, freq/clk_error)
        pwm.set_pwm_freq(freq/clk_error)
        for i in range(16):
                rospy.Subscriber("PWM_Channels/PulseTimeControl/CH"+str(i), Float64, time_callback, i, queue_size=10)
                rospy.Subscriber("PWM_Channels/DutyCycleControl/CH"+str(i), Float64, percent_callback, i, queue_size=10)

        rospy.spin()



if __name__ == '__main__':
    listener()
