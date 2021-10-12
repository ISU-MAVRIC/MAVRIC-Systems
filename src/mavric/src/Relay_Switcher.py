#!/usr/bin/env python3
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool

config = {
    "Lightbar": 1
}

def callback(data, args):
    print(args[1], data.data)
    #GPIO.output(args[1], data.data)

def talker():
        rospy.init_node('Relay_Switcher')
        GPIO.setmode(GPIO.BCM)
        for relay_name, relay_pin in config.items():
            rospy.Subscriber('/Relay/' + relay_name, Bool, callback, (relay_name, relay_pin))
            GPIO.setup(relay_pin, GPIO.OUT)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
                r.sleep()

if __name__ == '__main__':
    talker()
