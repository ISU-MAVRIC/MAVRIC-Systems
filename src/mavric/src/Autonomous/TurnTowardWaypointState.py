import auto_globals
from geographiclib.geodesic import Geodesic
from math import copysign

from StateMachine import State

#define functions
def get_inner_power(outer_power, radius):
    return (radius / (radius + auto_globals.Rover_Width)) * outer_power

class TurnTowardWaypoint(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
    
    def enter(self):
        self.angular_error = auto_globals.ANG_ERROR_THRESHOLD * 2   #arbitrary, default

    def run(self):
        auto_globals.prev_fix_time = auto_globals.fix_time  #update in gps_cb

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

	#TODO: turning +2 is better than -358

        #apply power based on heading
        left_power = 0
        right_power = 0
        
        if(self.angular_error < 0):
            #turn left
            left_power = get_inner_power(auto_globals.Scale, auto_globals.Rover_MinTurnRadius)
            right_power = auto_globals.Scale

        if(self.angular_error > 0):
            #turn right
            left_power = auto_globals.Scale
            right_power = get_inner_power(auto_globals.Scale, auto_globals.Rover_MinTurnRadius)

        auto_globals.drive_pub.publish(left_power * 100, right_power * 100)

        #remember angular error for the next cycle
        auto_globals.prev_angular_error = self.angular_error

	auto_globals.debug_pub.publish("ang error, left power, right power")
	auto_globals.debug_pub.publish("%d, %d, %d" % (self.angular_error, left_power, right_power))

    def next(self):
        if(not auto_globals.enabled or not auto_globals.good_fix or auto_globals.fix_timeout):
            return self._stateMachine.idle

        if(abs(self.angular_error) < auto_globals.ANG_ERROR_THRESHOLD):
            return self._stateMachine.driveTowardWaypoint

        return self._stateMachine.turnTowardWaypoint
