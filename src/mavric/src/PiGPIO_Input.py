#!/usr/bin/env python
# Reads a GPIO pin and publishes its state to the ROS as a boolean.

# Parameters
#   pin - The pin to be read (BCM numbering, see pinout.xyz or something)
#   rate - How often to read the pin and publish, defaults to 10 Hz.

# Topics:
#   State - Publication: The state of the pin (True = high voltage, False = low voltage)

import rospy
from std_msgs.msg import Bool
import wiringpi

def talker():
        rospy.init_node('PiInput')
        pub = rospy.Publisher('State', Bool, queue_size=10, latch=True)
        rate = rospy.get_param('~rate', 10)
        pin = rospy.get_param('~pin', -1)
        if pin == -1:
                raise ValueError('~pin not set')
        r = rospy.Rate(rate)
        wiringpi.wiringPiSetupGpio()
        wiringpi.pinMode(pin,0)
        while not rospy.is_shutdown():
                pinstate = wiringpi.digitalRead(pin)
                pub.publish(pinstate)
                r.sleep()
                
if __name__ == '__main__':
    talker()
