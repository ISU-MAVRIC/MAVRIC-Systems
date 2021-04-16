#!/usr/bin/env python
import rospy
import Jetson.GPIO as GPIO
import time
from std_msgs.msg import Int32

GPIO.setmode(GPIO.BOARD) 
GPIO.setup(38, GPIO.IN) #pin 38
GPIO.setup(40, GPIO.IN) #pin 40

GPIO.add_event_detect(38, GPIO.RISING)
GPIO.add_event_detect(40, GPIO.RISING)
atime = none
btime = none
counter = 0

    def talker():
        pub = rospy.Publisher('chatter', Int32, queue_size=10)
        rospy.init_node('encoderFB', anonymous=True)
        while not rospy.is_shutdown():

            if GPIO.event_detected(38):
                atime = time.perf.counter()
            else:
                atime = none

            if GPIO.event_detected(40):
                btime = time.perf.counter()
            else:
                btime = none

            if atime == None or btime == None:
                counter = counter
            elif atime < btime:
                counter += 1
            elif btime < atime:
                counter -= 1

            pub.publish(counter)
            
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

