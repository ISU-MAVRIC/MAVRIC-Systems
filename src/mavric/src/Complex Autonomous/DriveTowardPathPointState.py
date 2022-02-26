import complex_globals as g
from geographiclib.geodesic import Geodesic
from math import copysign
import math as m

from StateMachine import State
from driver import Driver

class DriveTowardPathPoint(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
        self.DriveLib = Driver()

    def enter(self):
        self.linear_error = g.LIN_ERROR_THRESHOLD * 2
        #set first waypoint in array as target
        self.tgt = [0, 0]
        self.tgt[0] = g.pathpoints["position"][g.pathpoint_num][0]
        self.tgt[1] = g.pathpoints["position"][g.pathpoint_num][1]

    def run(self):
        g.prev_fix_time = g.fix_time

        #capture position in case it changes later
        pos = g.position

        #https://geographiclib.sourceforge.io/html/python/code.html#geographiclib.geodesic.Geodesic.Inverse
        #The inverse function returns an array of data regarding the soltion of the geodesic problem, more can be read about it above
        #solve the geodesic problem corresponding to these lat-lon values
        #   assumes WGS-84 ellipsoid model
        geod = Geodesic.WGS84.Inverse(pos[1], pos[0], self.tgt[1], self.tgt[0])

        #get linear error in meters
        self.linear_error = geod['s12']

        #todo, investigate what needs to happen here if we are messing with the heading this early
        #get angular error in CW degrees (account for 360 rollover)
        self.angular_error = (((geod['azi1'] - g.heading) + 360) % 360) if (((geod['azi1'] - g.heading) + 360) % 360) < 180 else -(((g.heading - geod['azi1']) + 360) % 360)


        #Check speed based on distance, probably some hard coded number
        #get values from some functions that do checking based on the offsets
        #remaining distance in meters
        decel_dist = 10        
        
        #Calculate linear velocity
        #Based on linear offset, output scaled max velocity        
        
        
        #shifts velocity by c, on the x axis
        #directly correlates by 1 meter
        #this could also help if rover doesn't seem to be slowing enough before a stop
        c = 7
        
        #controls the top end of the curve. top velocity, presumably 0 -> 100
        #TODO check range, starting with 40 percent power, if 100 is max velocity
        b = g.pathpoints["speed"][g.pathpoint_num]
        
        #r controls when the deccel starts, 0.75 is around 10 - 15m
        #0.3 is around 20m
        #note that this literally stretches the curve, the funciton then may need to be shifted
        r = 0.75
        
        #do speed input based on remaining distance, use r to adjust when the algo kicks in
        if self.linear_error < 10:
            velocity = (b * m.exp(r * self.linear_error)) / ( m.exp(c * r) + m.exp(r * self.linear_error))
        else:
            velocity = b

        #Calculate wheel angle
        #Based on angular offset, turning angle based severity of offset
        
        wheel_angle = self.angular_error
        
        
        #given some steering data, return vector of motor data
        driveVals = self.DriveLib.v_car_steer(velocity, wheel_angle)

        #publish drive data, given from DriveLib
        #publish first 6 wheel speed values    
        g.drive_pub.publish(driveVals[0], driveVals[1], driveVals[2], driveVals[3], driveVals[4], driveVals[5])
        #publish 4 wheel steer parameters
        g.steer_pub.publish(driveVals[6], driveVals[7], driveVals[8], driveVals[9])


        #remember linear error for the next cycle
        g.prev_linear_error = self.linear_error

        g.debug_pub.publish("lin error, angular error")
        g.debug_pub.publish("%d, %d" % (self.linear_error, self.angular_error))

    def next(self):
        if(not g.enabled or not g.good_fix or g.fix_timeout):
            return self._stateMachine.idle
            
        if(self.linear_error <= g.LIN_ERROR_THRESHOLD):
            if self.tgt == g.waypoints[0]:
                return self._stateMachine.reachedWaypoint
            else:
                return self._stateMachine.nextPathPoint
        
        if self.tgt != g.pathpoints["position"][g.pathpoint_num]:
            g.pathpoint_num += -1
            return self._stateMachine.nextPathPoint
        
        return self._stateMachine.driveTowardPathPoint
