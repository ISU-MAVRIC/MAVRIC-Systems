import complex_globals as g
from StateMachine import State
from driver import Driver

class BankTowardPathPoint(State):

    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
        self.DriveLib = Driver()

    def enter(self):
        self.linear_error = g.LIN_ERROR_THRESHOLD * 2
        self.tgt = [0, 0]
        self.tgt[0] = g.pathpoints["position"][g.pathpoint_num][0]
        self.tgt[1] = g.pathpoints["position"][g.pathpoint_num][1]
        self.desired_heading = g.pathpoints["heading"][g.pathpoint_num]
        self.turn_radius = g.pathpoints["radius"][g.pathpoint_num]
    
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
        self.angular_error = (((self.desired_heading - g.heading) + 360) % 360) if (((self.desired_heading - g.heading) + 360) % 360) < 180 else -(((self.desired_heading - geod['azi1']) + 360) % 360)

        #given some steering data, return vector of motor data
        driveVals = self.DriveLib.r_car_steer(velocity, self.turn_radius)

        #publish drive data, given from DriveLib
        #publish first 6 wheel speed values    
        g.drive_pub.publish(driveVals[0], driveVals[1], driveVals[2], driveVals[3], driveVals[4], driveVals[5])
        #publish 4 wheel steer parameters
        g.steer_pub.publish(driveVals[6], driveVals[7], driveVals[8], driveVals[9])


        #remember linear error for the next cycle
        g.prev_linear_error = self.linear_error

    def next(self):
        if(not g.enabled or not g.good_fix or g.fix_timeout):
            return self._stateMachine.idle
            
        if(self.linear_error <= g.LIN_ERROR_THRESHOLD and self.angular_error < g.ANG_ERROR_THRESHOLD):
            if self.tgt == g.waypoints[0]:
                return self._stateMachine.reachedWaypoint
            else:
                return self._stateMachine.nextPathPoint
        
        return self._stateMachine.bankTowardPathPoint
