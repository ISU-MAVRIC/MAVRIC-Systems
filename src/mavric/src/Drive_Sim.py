#!/usr/bin/env python3
import rospy
import time
import math

from mavric.msg import GPS, Drivetrain, Steertrain
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

from geographiclib.geodesic import Geodesic

lf = 0
rf = 0
strlf = 0
strrf = 0

def drive_cb(data):
    global lf, rf
    strlf = data.lf
    strrf = data.rf
    
    if lf > 100:
        lf = 100
    elif lf < -100:
        lf = -100
    if rf > 100:
        rf = 100
    elif rf < -100:
        rf = -100
    
def steer_cb(data):
    global strlf, strrf
    strlf = data.strLf
    strrf = data.strRf
    
    if strlf > 90:
        strlf = 90
    elif strlf < -90:
        strlf = -90
    if strrf > 90:
        strrf = 90
    elif strrf < -90:
        strrf = -90

# talker function
def talker():
    global lf, rf, strlf, strrf, heading, lon, lat
    rospy.init_node('Drive_Simulator')
    dc_to_mps = rospy.get_param('~dc_mps', 2.4)/100
    lon = rospy.get_param('~init_lon', -93.646558)
    lat = rospy.get_param('~init_lat', 42.026272)
    heading = rospy.get_param('~init_heading', 0)
    L = rospy.get_param('~wheel_length', 37.5/39.37)
    W = rospy.get_param('~wheel_width', 28.4/39.37)
    point_r = math.sqrt(math.pow(L/2, 2)+math.pow(W/2, 2))

    #subscibers to drive, steer, and pubs to imu, gps
    drive_sub = rospy.Subscriber('Drive_Train', Drivetrain, drive_cb, queue_size=10)
    steer_sub = rospy.Subscriber('Steer_Train', Steertrain, steer_cb, queue_size=10)
    imu_pub = rospy.Publisher("FusedAngle", Vector3, queue_size=10)
    pub = rospy.Publisher('GPS_Data', GPS, queue_size=10, latch=True)
    pub_gps_fix = rospy.Publisher('GPS_Fix', BOOL_MSG, queue_size=10, latch=True)
    r = rospy.Rate(5)

    prev_time = time.time()
    prev_lon = lon
    prev_lat = lat
    d = 0

    #start main loop
    while not rospy.is_shutdown():
        # calculate linear distance and heading
        # if going straight
        if strlf == 0 and strrf == 0:
            d = lf*dc_to_mps*(time.time()-prev_time)
        
        # if point steer
        elif strlf == -strrf:
            d = 0
            heading += math.degrees(lf*dc_to_mps*(time.time()-prev_time)/point_r)
            heading %= 360
        
        # if dynamic steer
        else:
            if strlf > strrf:
                in_angle = strlf
                vel = lf
            else:
                in_angle = strrf
                vel = rf
            
            d = vel*dc_to_mps*(time.time()-prev_time)
            r = L/(2*math.sin(in_angle))
            heading += math.degrees(d/r)
            heading %= 360
        
        # calc gps vales
        # calculate new lon and lat
        geod = Geodesic.WGS84.Direct(lat, lon, theta, d)

        # get velocity
        gps_vel = d/(time.time()-prev_time)

        # get heading
        geod2 = Geodesic.WGS84.Inverse(lat, lon, geod["lat2"], geod["lon2"])
        gps_heading = geod2['azi1']

        # set new lat and lon
        lon = geod["lon2"]
        lat = geod["lat2"]

        # publish data
        pub_gps_fix.publish(True)
        pub.publish(geod["lat2"], geod["lon2"], gps_vel, gps_heading, 0, 0, 0, 0)
        imu_pub.publish(0, 0, gps_heading)


# main loop
if __name__ == '__main__':
    talker()