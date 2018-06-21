#!/usr/bin/env python

import serial
import time
import rospy

from std_msgs.msg import String
from math import sin, cos, atan2, radians, degrees
from geographiclib.geodesic import Geodesic
from pid_controller.pid import PID

from gps import GPS

#declare constants
LIN_ERROR_THRESHOLD = 5		#arbitrary, meters
ANG_ERROR_THRESHOLD = 10
UPDATE_INTERVAL = 1
FIX_TIMEOUT_THRESHOLD = UPDATE_INTERVAL * 5

#PID constants
L_P = 0.0
L_I = 0.0
L_D = 0.0

A_P = 0.0
A_I = 0.0
A_D = 0.0

#declare variables
linear_error = LIN_ERROR_THRESHOLD + 1
prev_linear_error = linear_error

angular_error = ANG_ERROR_THRESHOLD + 1
prev_angular_error = angular_error

prev_fix_time = 0

need_tgt = True
fix_timeout = False

linearPID = PID(L_P, L_I, L_D)
angularPID = PID(A_P, A_I, A_D)

gps = GPS('COM5', 9600, 1)

""" -MAIN LOOP- """
def talker():
    global need_tgt, fix_timeout
    global prev_fix_time, prev_linear_error, prev_angular_error

    #create ROS publisher
    pub = rospy.Publisher("/Drive_Train", String, queue_size=10)
    rospy.init_node("DTP", anonymous=True)
    
    #open serial port and turn on GPS power
    #gps.begin()

    while not rospy.is_shutdown():
        #update gps data
        #gps.update()

        #if(hms_to_s(gps.time) - prev_fix_time < UPDATE_INTERVAL):
        #    continue
        
        time.sleep(UPDATE_INTERVAL)
        
        #get new gps target from base station, if necessary, and split it by comma
        #   !NOTE may or may not be an NMEA sentence, assuming lat-lon pair
        if need_tgt:
            tgt_sentence = raw_input("target lat, lon: ").split(",")
            need_tgt = False
        
        #only perform GPS calculations if GPS data is valid
        #   !NOTE may want to check number of satellites > 4 instead?
        if True: #gps.good_fix
            #update fix timer
            #   !NOTE may want to depend on an internal clock instead of satellite time?
            #prev_fix_time = hms_to_s(gps.time)
            fix_timeout = False

            #input position data
            pos = [42.0308, -93.6319]
            head = 0.0
            #pos = [gps.latitude, gps.longitude]
            #head = gps.heading
            tgt = [0, 0]

            tgt[0] = float(tgt_sentence[0])
            tgt[1] = float(tgt_sentence[1])

            #solve the geodesic problem corresponding to these lat-lon values
            #   assumes WGS-84 ellipsoid model
            geod = Geodesic.WGS84.Inverse(pos[0], pos[1], tgt[0], tgt[1])
            
            #get linear error in meters
            linear_error = geod['s12']

            #calculate angle between pos and tgt
            #   assumes great-circle (spherical Earth) model
            
            #convert lat-lon to radians
            pos = [radians(pos[0]), radians(pos[1])]
            tgt = [radians(tgt[0]), radians(tgt[1])]

            #calculate heading to target, then angular error
            kY = cos(tgt[0]) * sin(tgt[1] - pos[1])
            kX = (cos(pos[0]) * sin(tgt[0])) - (sin(pos[0]) * cos(tgt[0]) * cos(tgt[1] - pos[1]))

            tgt_head = degrees(atan2(kY, kX))
            angular_error = tgt_head - head

            #convert clockwise angle to counterclockwise
            #NOTE should do this to an abs angle, not the error!
            #angular_error = 360 - angular_error
            
        else:
            #if GPS data is not valid, use previously calculated values
            linear_error = prev_linear_error
            angular_error = prev_angular_error

            #if GPS data has been invalid for a while, force motors to stop
            if hms_to_s(gps.time) - prev_fix_time > FIX_TIMEOUT_THRESHOLD:
                fix_timeout = True

        #check if the target position has been reached
        if abs(linear_error) < LIN_ERROR_THRESHOLD:
            need_tgt = True

        #pass linear and angular error to separate PID controllers
        #   !NOTE may need to use modulus to fix angular rollover
        linear_power = linearPID(feedback=linear_error)
        angular_power = angularPID(feedback=angular_error)

        #fuse PID controller outputs to obtain fractional motor power (-1.0 - 1.0)
        #something along these lines, may have to swap which is + and which is -
        left_power = linear_power + angular_power;
        right_power = linear_power - angular_power;

        #debug, apply power based on heading
        print(angular_error)
        
        if(angular_error < 0):
            print("Turning left...")
            left_power = -0.5
            right_power = 0.5

        if(angular_error > 0):
            print("Turning right...")
            left_power = 0.5
            right_power = -0.5

        if(abs(angular_error) < ANG_ERROR_THRESHOLD and
           abs(linear_error) > LIN_ERROR_THRESHOLD):
            print("Driving forward...")
            left_power = 0.5
            right_power = 0.5

        #override motor power if GPS data has been bad for a while
        if fix_timeout:
            left_power = 0.0
            right_power = 0.0

        #convert from fractional motor power to command string
        pub.publish("D" + frac_to_cs(left_power) + frac_to_cs(right_power))

        #remember linear and angular error for the next cycle
        prev_linear_error = linear_error
        prev_angular_error = angular_error

""" -HELPER FUNCTION DEFINITIONS- """
#convert from hours-minutes-seconds to seconds
def hms_to_s(time):
    h = time[0]
    m = time[1]
    s = time[2]
    
    return (h * 60 * 60) + (m * 60) + s;

#convert from fractional motor power (-1.0 - 1.0) to command string (00 - 99)
def frac_to_cs(f):
    k = ((f * 100) / 2) + 50
    
    #clip value from 100 to 99, scale instead?
    if k > 99:
        k = 99

    #return k as a string for concatenation
    return str(int(k))

""" -MAIN FUNCTION- """
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
