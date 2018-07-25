#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def talker():

        rospy.init_node('TempMonitoring')
        pub = rospy.Publisher("Temperature", Float32, queue_size=10)
        r = rospy.Rate(1);
        while not rospy.is_shutdown():
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as content_file:
                        tempString = content_file.read()
                pub.publish(float(tempString)/1000)
                r.sleep()
        

if __name__ == '__main__':
    talker()
