import complex_globals as g
from geographiclib.geodesic import Geodesic
from math import copysign
import time

from driver import Driver

from StateMachine import State


#On inital trek to a waypoint, turn twords the waypoint, then advance the state

class TurnTowardPathPoint(State):

    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
        self.D = Driver()
    

    def get_angular_error(self):
        return (((self.desired_heading - g.heading) + 360) % 360) if (((self.desired_heading - g.heading) + 360) % 360) < 180 else -(((g.heading - self.desired_heading) + 360) % 360)



    def get_ramped_turn_speed(self):
        turn_speed = ((g.ANG_POINT_STEER_MAX - g.ANG_POINT_STEER_MIN) / 90) * abs(self.get_angular_error()) + g.ANG_POINT_STEER_MIN
        turn_speed = turn_speed if turn_speed < g.ANG_POINT_STEER_MAX else g.ANG_POINT_STEER_MAX
        return copysign(turn_speed, self.get_angular_error())
    

    def enter(self):

        self.angular_error = g.ANG_ERROR_THRESHOLD * 2   #arbitrary, default

        lf, lm, lb, rf, rm, rb, lfs, lbs, rfs, rbs = self.D.v_point_steer(0)
        g.drive_pub.publish(0,0,0,0,0,0)
        g.steer_pub.publish(lfs, lbs, rfs, rbs)

        time.sleep(1)

        #set first waypoint in array as target
        self.tgt = [0, 0]
        self.tgt[0] = g.pathpoints["position"][g.pathpoint_num][0]
        self.tgt[1] = g.pathpoints["position"][g.pathpoint_num][1]

        #capture position in case it changes later
        pos = g.position

        #solve the geodesic problem corresponding to these lat-lon values
        #   assumes WGS-84 ellipsoid model
        geod = Geodesic.WGS84.Inverse(pos[1], pos[0], self.tgt[1], self.tgt[0])
        self.desired_heading = geod['azi1']

        self.start_time = time.time()



    def run(self):
        g.prev_fix_time = g.fix_time  #update in gps_cb

        #auto_globals.debug_pub.publish(str(tgt))
        #auto_globals.debug_pub.publish(str(pos))
        #auto_globals.debug_pub.publish(str(geod))

        if abs(self.get_angular_error()) > g.ANG_ERROR_THRESHOLD and time.time() < (self.start_time + g.ANG_POINT_STEER_TIMEOUT):
            g.debug_pub.publish(str(self.get_ramped_turn_speed()))
            lf, lm, lb, rf, rm, rb, lfs, lbs, rfs, rbs = self.D.v_point_steer(self.get_ramped_turn_speed())
            g.debug_pub.publish(str(self.get_angular_error()))
            g.debug_pub.publish("a"+str(self.desired_heading))
            g.drive_pub.publish(lf, lm, lb, rf, rm, rb)
            g.steer_pub.publish(lfs, lbs, rfs, rbs)
        else:
            g.drive_pub.publish(0,0,0,0,0,0)
            g.steer_pub.publish(0,0,0,0)


    def next(self):
        if(not g.enabled or not g.good_fix or g.fix_timeout):
            return self._stateMachine.idle
        
        if(self.tgt != g.pathpoints["position"][g.pathpoint_num]):
            g.pathpoint_num += -1
            return self._stateMachine.nextPathPoint

        if(abs(self.get_angular_error()) < g.ANG_ERROR_THRESHOLD):
            return self._stateMachine.driveTowardWaypoint

        return self._stateMachine.turnTowardWaypoint

