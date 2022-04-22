#!/usr/bin/env python3
import rospy

from mavric.msg import LED
from LED import LightPole

light_pole = LightPole()

def light_pole_callback(data):
    light_pole.setColor(data.color)
    light_pole.setBrightness(data.brightness)
    if (data.toggle != light_pole.isToggled()):
        light_pole.toggle()
    
def listener():
    rospy.init_node('LED_Control', anonymous=True)
    rospy.Subscriber("/indicators/light_pole", LED, light_pole_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass