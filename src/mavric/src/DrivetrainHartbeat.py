#!/usr/bin/env python

import rospy
from mavric.msg import Drivetrain

def callback(data):
        global frame
        frame = 0

def talker():
        global frame
        rospy.init_node('DrivetrainHartbeat')
        timeout = rospy.get_param('~timeout', 10)
        rospy.Subscriber('Drive_Train', Drivetrain, callback, queue_size=10)
        pub = rospy.Publisher('Drive_Train', Drivetrain, queue_size=10)
        frame = 0
        r = rospy.Rate(10/timeout)
        while not rospy.is_shutdown():
                r.sleep()
                frame = frame + 1
                if (frame > 10):
                        pub.publish(0,0)

if __name__ == '__main__':
    talker()
