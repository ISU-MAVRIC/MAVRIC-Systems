#!/usr/bin/env python

import serial
import time
import rospy

from std_msgs.msg import String
from mavric.msg import Autonomous, Waypoint, Drivetrain, GPS

from math import sin, cos, atan2, radians, degrees
from geographiclib.geodesic import Geodesic
from simple_pid import PID

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

waypoints = []

position = [0, 0]
prev_position = [0, 0]

fix_time = 0
prev_fix_time = 0

enabled = True
good_fix = False
fix_timeout = False

linearPID = PID(L_P, L_I, L_D, setpoint=0)  #minimize error
angularPID = PID(A_P, A_I, A_D, setpoint=0) #minimize error

""" -HELPER FUNCTION DEFINITIONS- """
#convert from hours-minutes-seconds to seconds
def hms_to_s(h,m,s):
    return (h * 60 * 60) + (m * 60) + s;

def get_inner_power(outer_power, radius):
    return (radius / (radius + Rover_Width)) * outer_power


def cmd_cb(data):
    #enable/disable autonomous, forget current waypoints
    if data.command == 'E':
        enabled = True

    elif data.command == 'D':
        enabled = False

    elif data.command == 'F':
        waypoints = []

def waypoint_cb(data):
    #add new waypoint to list
    waypoints.append([data.latitude, data.longitude])

def gps_cb(data):
    #how often does the gps publish? do we have to compare values here?
    global position, prev_position, fix_time, good_fix, heading

    prev_position = position
    position = [data.latitude, data.longitude]
    fix_time = hms_to_s(data.timeh, data.timem, data.times)
    good_fix = data.good_fix
    heading = data.heading

""" -MAIN LOOP- """
def talker():
    global enabled, fix_timeout, debug_pub, good_fix
    global position, heading
    global prev_fix_time, prev_linear_error, prev_angular_error
    global Scale, Rover_Width, Rover_MinTurnRadius

    #setup ROS node
    rospy.init_node("ANS")
    
    pub = rospy.Publisher("Drive_Train", Drivetrain, queue_size=10)
    debug_pub = rospy.Publisher("Autonomous_Debug", String, queue_size=10)

    cmd_sub = rospy.Subscriber("Autonomous", Autonomous, cmd_cb, queue_size=10)
    way_sub = rospy.Subscriber("Next_Waypoint", Waypoint, waypoint_cb, queue_size=10)
    gps_sub = rospy.Subscriber("/GPS_Data", GPS, gps_cb, queue_size=10)

    Scale = rospy.get_param("~Range", 0.5)
    Rover_Width = rospy.get_param("~Rover_Width", 1)
    Rover_MinTurnRadius = rospy.get_param("~Rover_MinTurnRadius", 2)

    rate = rospy.Rate(2)    #2 Hz

    while not rospy.is_shutdown():
        #wait for enable
        if not enabled:
            rate.sleep()
            continue
        
        #wait for a waypoint from base station before doing anything
        if len(waypoints) == 0:
            pub.publish(0, 0)
            rate.sleep()
            continue
        
        #only perform GPS calculations if GPS data is valid
        if good_fix:

            #update fix timer
            #   !NOTE may want to depend on an internal clock instead of satellite time?
            #   !NOTE this is based on the GPS module, not the ROS node
            prev_fix_time = fix_time
            fix_timeout = False

            #set first waypoint in array as target
            tgt = [0, 0]
            tgt[0] = waypoints[0][0]
            tgt[1] = waypoints[0][1]

            #capture position in case it changes later
            pos = position

            #solve the geodesic problem corresponding to these lat-lon values
            #   assumes WGS-84 ellipsoid model
            geod = Geodesic.WGS84.Inverse(pos[0], pos[1], tgt[0], tgt[1])
            
            #get linear error in meters
            linear_error = geod['s12']
            debug_pub.publish(str(linear_error))

            #calculate angle between pos and tgt
            #   assumes great-circle (spherical Earth) model
            
            #convert lat-lon to radians
            pos = [radians(pos[0]), radians(pos[1])]
            tgt = [radians(tgt[0]), radians(tgt[1])]

            #calculate heading to target, then angular error
            kY = cos(tgt[0]) * sin(tgt[1] - pos[1])
            kX = (cos(pos[0]) * sin(tgt[0])) - (sin(pos[0]) * cos(tgt[0]) * cos(tgt[1] - pos[1]))
            
            tgt_head = degrees(atan2(kY, kX))
            tgt_head = (tgt_head + 360) % 360
            angular_error = tgt_head - heading

            debug_pub.publish(str(tgt_head))
            debug_pub.publish(str(head))
            #convert clockwise angle to counterclockwise
            #NOTE should do this to an abs angle, not the error!
            #angular_error = 360 - angular_error
            
        else:
            #if GPS data is not valid, use previously calculated values
            linear_error = prev_linear_error
            angular_error = prev_angular_error

            #if GPS data has been invalid for a while, force motors to stop
            if fix_time - prev_fix_time > FIX_TIMEOUT_THRESHOLD:
                fix_timeout = True

        #check if the target position has been reached
        if abs(linear_error) < LIN_ERROR_THRESHOLD:
            waypoints.pop(0)
            debug_pub.publish("Reached Waypoint")

        #pass linear and angular error to separate PID controllers
        #   !NOTE may need to use modulus to fix angular rollover
        linear_power = linearPID(linear_error)
        angular_power = angularPID(angular_error)

        #fuse PID controller outputs to obtain fractional motor power (-1.0 - 1.0)
        #something along these lines, may have to swap which is + and which is -
        left_power = linear_power + angular_power;
        right_power = linear_power - angular_power;

        #debug, apply power based on heading
	debug_pub.publish(str(good_fix))
	debug_pub.publish(str(angular_error))
        debug_pub.publish(str(linear_error))
        
        if(angular_error < 0):
            print("Turning left...")
            left_power = get_inner_power(Scale, Rover_MinTurnRadius)
            right_power = Scale

        if(angular_error > 0):
            print("Turning right...")
            left_power = Scale
            right_power = get_inner_power(Scale, Rover_MinTurnRadius)

        if(abs(angular_error) < ANG_ERROR_THRESHOLD and
           abs(linear_error) > LIN_ERROR_THRESHOLD):
            print("Driving forward...")
            left_power = Scale
            right_power = Scale

        #override motor power if GPS data has been bad for a while
        if fix_timeout:
            left_power = 0.0
            right_power = 0.0

        #convert from fractional motor power to command
        pub.publish(left_power * 100, right_power * 100)

        #remember linear and angular error for the next cycle
        prev_linear_error = linear_error
        prev_angular_error = angular_error

        #wait for next iteration
        rate.sleep()

""" -MAIN FUNCTION- """
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
