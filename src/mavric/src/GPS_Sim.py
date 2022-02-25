#!/usr/bin/env python3
import rospy
import time

from mavric.msg import GPS, Drivetrain
from geometry_msgs.msg import Vector3

from geographiclib.geodesic import Geodesic

vel = 0
theta = 0
lon = 0
lat = 0

def drive_cb(data):
        global vel
        if data.lf > 100:
            vel = 100
        elif data.lf < -100:
            vel = -100
        elif data.lf*data.rf < 0:
            vel = 0
        else:
            vel = data.lf

def imu_cb(data):
    global theta
    theta = data.z

# talker function
def talker():
    global vel
    global theta
    global lon
    global lat
    rospy.init_node('GPS_Simulator')
    dc_to_mps = rospy.get_param('~dc_mps', 2.5)/100

    #subscibers to drive and imu
    rospy.Subscriber('Drive_Train', Drivetrain, drive_cb, queue_size=10)
    imu_sub = rospy.Subscriber("FusedAngle", Vector3, imu_cb, queue_size=10)
    pub = rospy.Publisher('GPS_Data', GPS, queue_size=10, latch=True)
    rate = rospy.Rate(5)

    prev_time = time.time()
    #start main loop
    while not rospy.is_shutdown():
        r.sleep()

        #calculate distance rover moved
        d = vel*dc_to_mps*(time.time()-prev_time)
        # set time for delta t
        prev_time = time.time()

        # calculate new lon and lat
        geod = Geodesic.WGS84.Direct(lat, lon, theta, d)

        # publish GPS data
        pub.publish(lat, lon, 0, 0, 0, 0, 0, 0, 0)

# main loop
if __name__ == '__main__':
    talker()