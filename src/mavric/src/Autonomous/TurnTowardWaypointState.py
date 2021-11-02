import auto_globals
from geographiclib.geodesic import Geodesic
from math import copysign
import time

from driver import Driver

from StateMachine import State



#On inital trek to a waypoint, turn twords the waypoint, then advance the state
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

        self.desired_heading = geod['azi1']

        start_time = time.time()

        while abs(get_angular_error()) > auto_globals.ANG_ERROR_THRESHOLD and time.time() < (start_time + auto_globals.ANG_POINT_STEER_TIMEOUT):
            lf, lm, lb, rf, rm, rb, lfs, lbs, rfs, rbs = Driver.v_point_steer(get_ramped_turn_speed())
            auto_globals.drive_pub.publish(lf, lm, lb, rf, rm, rb, lfs, lbs, rfs, rbs)

        auto_globals.drive_pub.publish(0,0,0,0,0,0,0,0,0,0)

    def next(self):
        if(not auto_globals.enabled or not auto_globals.good_fix or auto_globals.fix_timeout):
            return self._stateMachine.idle

        if(abs(self.angular_error) < auto_globals.ANG_ERROR_THRESHOLD):
            return self._stateMachine.driveTowardWaypoint

        return self._stateMachine.turnTowardWaypoint

    
    def get_angular_error(self):
        return_heading = self.desired_heading - auto_globals.heading
        return copysign(return_heading % 180, return_heading)


    def get_ramped_turn_speed(self):
        turn_speed = ((auto_globals.ANG_POINT_STEER_MAX - auto_globals.ANG_POINT_STEER_MIN) / 90) * abs(get_angular_error()) + auto_globals.ANG_POINT_STEER_MIN
        turn_speed = turn_speed if turn_speed < auto_globals.ANG_POINT_STEER_MAX else auto_globals.ANG_POINT_STEER_MAX
        return copysign(turn_speed, get_angular_error())
