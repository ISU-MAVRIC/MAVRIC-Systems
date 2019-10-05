import auto_globals
from geographiclib.geodesic import Geodesic
from math import copysign

from StateMachine import State

class DriveTowardWaypoint(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine

    def enter(self):
        self.linear_error = auto_globals.LIN_ERROR_THRESHOLD * 2

    def run(self):
        auto_globals.prev_fix_time = auto_globals.fix_time

        #set first waypoint in array as target
        tgt = [0, 0]
        tgt[0] = auto_globals.waypoints[0][0]
        tgt[1] = auto_globals.waypoints[0][1]

        #capture position in case it changes later
        pos = auto_globals.position

        #solve the geodesic problem corresponding to these lat-lon values
        #   assumes WGS-84 ellipsoid model
        geod = Geodesic.WGS84.Inverse(pos[0], pos[1], tgt[0], tgt[1])

        #get linear error in meters
        self.linear_error = geod['s12']

        #get angular error in CW degrees (account for 360 rollover)
        self.angular_error = geod['azi1'] - auto_globals.heading
	self.angular_error = copysign(abs(self.angular_error) % 360, self.angular_error)

        #apply power based on heading
        left_power = 0
        right_power = 0

        if(self.linear_error > auto_globals.LIN_ERROR_THRESHOLD):
            #drive forward
            left_power = auto_globals.Scale
            right_power = auto_globals.Scale

        auto_globals.drive_pub.publish(left_power * 100, right_power * 100)

        #remember linear error for the next cycle
        auto_globals.prev_linear_error = self.linear_error

	auto_globals.debug_pub.publish("lin error, left power, right power")
	auto_globals.debug_pub.publish("%d, %d, %d" % (self.linear_error, left_power, right_power))

    def next(self):
        if(not auto_globals.enabled or not auto_globals.good_fix or auto_globals.fix_timeout):
            return self._stateMachine.idle
            
        elif(self.linear_error <= auto_globals.LIN_ERROR_THRESHOLD):
            return self._stateMachine.reachedWaypoint

        return self._stateMachine.driveTowardWaypoint
