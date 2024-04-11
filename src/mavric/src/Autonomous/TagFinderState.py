import auto_globals
from StateMachine import State
from ArucoClass import Aruco
from driver import Driver
from math import copysign

class TagFinder(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
        self.Aruco = Aruco()
        self.D = Driver()

    def get_angular_error(self):
        return (((self.desired_heading - auto_globals.heading) + 360) % 360) if (((self.desired_heading - auto_globals.heading) + 360) % 360) < 180 else -(((auto_globals.heading - self.desired_heading) + 360) % 360)

    def get_ramped_turn_speed(self):
        turn_speed = ((auto_globals.ANG_POINT_STEER_MAX - auto_globals.ANG_POINT_STEER_MIN) / 90) * abs(self.get_angular_error()) + auto_globals.ANG_POINT_STEER_MIN
        turn_speed = turn_speed if turn_speed < auto_globals.ANG_POINT_STEER_MAX else auto_globals.ANG_POINT_STEER_MAX
        return copysign(turn_speed, self.get_angular_error())

    def enter(self):
        lf, lm, lb, rf, rm, rb, lfs, lbs, rfs, rbs = self.D.v_point_steer(0)
        auto_globals.drive_pub.publish(0,0,0,0,0,0)
        auto_globals.steer_pub.publish(lfs, lbs, rfs, rbs)
    
    def run(self):
        

    def next(self):

        return self._stateMachine.TagFinderState
