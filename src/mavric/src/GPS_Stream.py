#!/usr/bin/env python

import rospy
import mavric.msg
from gps import GPS

def talker():

        rospy.init_node('GPS_Streamer')
        pub = rospy.Publisher("GPS_Data", mavric.msg.GPS, queue_size=10)
        r = rospy.Rate(10);
        gps = GPS('/dev/serial0')
        gps.begin()
        while not rospy.is_shutdown():
                print(gps._data)
                gps.update()
                pub.publish(gps.latitude, gps.longitude, gps.altitude, gps.speed, gps.heading, gps.satellites)
                r.sleep()
        

if __name__ == '__main__':
    talker()
