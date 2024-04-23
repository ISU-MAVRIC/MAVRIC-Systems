import auto_globals
from ArucoClass import Aruco
from math import copysign
import math as m
import time

from StateMachine import State
from driver import Driver

class DriveTowardTag(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
        self.Aruco = Aruco()
        self.DriveLib = Driver()
        self.failures = 0

    def enter(self):
        self.linear_error = auto_globals.LIN_ERROR_THRESHOLD * 2

    def run(self):
        # get frame data, process markers, and calculate distance
        frame = self.Aruco.grab_frame()
        markers = self.Aruco.get_markers(frame)
        self.linear_error = self.Aruco.get_dist(frame)
        self.angular_error = self.Aruco.get_angles(markers)
        if len(self.linear_error) > 0:
            rem_dist = self.linear_error[0]
            turn_error = self.angular_error[0]
            
            #controls the top end of the curve. top velocity, presumably 0 -> 100
            b = 40
            
            #do speed input based on remaining distance, use r to adjust when the algo kicks in
            if rem_dist < 5:
                velocity = b/2
            else:
                velocity = b

            #Calculate wheel angle
            #Based on angular offset, turning angle based severity of offset
            
            #TODO should the wheel turn angle just literally be the offset? or the opposite of the offset?
            #TODO, do we need to ramp this value somehow, or scale it base on the offset?
            wheel_angle = turn_error
            
            
            #given some steering data, return vector of motor data
            driveVals = self.DriveLib.v_car_steer(velocity, wheel_angle)

            #publish drive data, given from DriveLib
            #publish first 6 wheel speed values    
            auto_globals.drive_pub.publish(driveVals[0], driveVals[1], driveVals[2], driveVals[3], driveVals[4], driveVals[5])
            #publish 4 wheel steer parameters
            auto_globals.steer_pub.publish(driveVals[6], driveVals[7], driveVals[8], driveVals[9])
        else:
            auto_globals.drive_pub.publish(0,0,0,0,0,0)

        #remember linear error for the next cycle
        auto_globals.prev_linear_error = self.linear_error

        auto_globals.debug_pub.publish("Tag linear error | Aruco Failures")
        auto_globals.debug_pub.publish(str(self.linear_error) + " " + str(self.failures))

    def next(self):
        frame = self.Aruco.grab_frame()
        markers = self.Aruco.get_markers(frame)
        if len(markers[0]) == 0:
            self.failures = self.failures + 1
            if self.failures > 4:
                return self._stateMachine.tagFinder
            else:
                return self._stateMachine.driveTowardTag
        
        if(auto_globals.prev_linear_error < self.linear_error):   # if the rover skips the waypoint and remaining distance starts increasing
            return self._stateMachine.tagFinder
        
        elif(not auto_globals.enabled or not auto_globals.good_fix or auto_globals.fix_timeout):
            return self._stateMachine.idle
            
        elif(self.linear_error <= auto_globals.LIN_ERROR_THRESHOLD*1.5):
            return self._stateMachine.reachedWaypoint
        else:
            self.failures = 0
            return self._stateMachine.driveTowardTag
