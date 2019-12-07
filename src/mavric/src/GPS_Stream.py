#!/usr/bin/env python
# Reads from the GPS pHAT and publishes the data to the GPS_Data topic.

# Topics:
#   GPS_Data - Publications: Outputs the data from the GPS module at about 10 Hz

import rospy
import mavric.msg
from gps import GPS

def talker():

        rospy.init_node('GPS_Streamer')
        pub = rospy.Publisher("GPS_Data", mavric.msg.GPS, queue_size=10, latch=True)
        r = rospy.Rate(10);
        gps = GPS('/dev/ttyS0')
        gps.begin()
        while not rospy.is_shutdown():
                print(gps._data)
                gps.update()
                h,m,s =  gps.time;
                pub.publish(gps.good_fix, gps.latitude, gps.longitude, gps.altitude, gps.speed, gps.heading, gps.satellites, h, m, s)
                r.sleep()
        

if __name__ == '__main__':
    talker()
