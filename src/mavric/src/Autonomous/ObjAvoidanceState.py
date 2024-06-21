import auto_globals, time
from math import copysign
from StateMachine import State
from driver import Driver

'''
THIS WAS NEVER USED BY THE MAVRIC 2024 TEAM.
Object detection was not needed during competition.
'''

class ObjAvoidance(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
        self.D = Driver()
        self.driveSpeed = 50

    def get_angular_error(self):
        return (((self.desired_heading - auto_globals.heading) + 360) % 360) if (((self.desired_heading - auto_globals.heading) + 360) % 360) < 180 else -(((auto_globals.heading - self.desired_heading) + 360) % 360)
    
    def get_ramped_turn_speed(self):
        turn_speed = ((auto_globals.ANG_POINT_STEER_MAX - auto_globals.ANG_POINT_STEER_MIN) / 90) * abs(self.get_angular_error()) + auto_globals.ANG_POINT_STEER_MIN
        turn_speed = turn_speed if turn_speed < auto_globals.ANG_POINT_STEER_MAX else auto_globals.ANG_POINT_STEER_MAX
        return copysign(turn_speed, self.get_angular_error())

    def enter(self):
        auto_globals.drive_pub.publish(0,0,0,0,0,0)
        auto_globals.steer_pub.publish(0,0,0,0)
        time.sleep(1)

    def run(self):
        if auto_globals.usLeft == True:
            self.desired_heading = (auto_globals.heading + 45 + 360) % 360
            while abs(self.get_angular_error()) > auto_globals.ANG_ERROR_THRESHOLD:
                lf,lm,lb,rf,rm,rb,lfs,lbs,rfs,rbs = self.D.v_point_steer(self.get_ramped_turn_speed())
                auto_globals.steer_pub.publish(lfs,lbs,rfs,rbs)
                auto_globals.drive_pub.publish(lf,lm,lb,rf,rm,rb)

        auto_globals.steer_pub.publish(0,0,0,0)
        auto_globals.drive_pub.publish(0,0,0,0,0,0)

        for i in range(5):
            auto_globals.drive_pub.publish(self.driveSpeed,self.driveSpeed,self.driveSpeed,self.driveSpeed,self.driveSpeed,self.driveSpeed)
            if auto_globals.usLeft or auto_globals.usMid or auto_globals.usRight:
                break
            time.sleep(0.5)
        
        auto_globals.drive_pub.publish(0,0,0,0,0,0)
        auto_globals.steer_pub.publish(0,0,0,0)

    def next(self):
        if(not auto_globals.usLeft and not auto_globals.usMid and not auto_globals.usRight):
            return self._stateMachine.turnTowardWaypoint
        else:
            return self._stateMachine.objAvoidance