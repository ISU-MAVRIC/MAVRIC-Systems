#!/usr/bin/env python3
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool

config = {
    "light_pole/red": 31,
    "light_pole/blue": 33,
    "light_pole/green": 35,
    "light_pole/buzzer": 37
}

def callback(data, args):
    GPIO.output(args[1], data.data)

def talker():
        rospy.init_node('Relay_Switcher')
        GPIO.setmode(GPIO.BOARD)
        for relay_name, relay_pin in config.items():
            GPIO.setup(relay_pin, GPIO.OUT)
            rospy.Subscriber('/relay/' + relay_name, Bool, callback, (relay_name, relay_pin))
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
                r.sleep()
        GPIO.cleanup()

if __name__ == '__main__':
    talker()
