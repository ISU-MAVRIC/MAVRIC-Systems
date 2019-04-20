import auto_globals

from StateMachine import State

class Idle(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
    
    def enter(self):
        auto_globals.drive_pub.publish(0, 0)

    def run(self):
        pass

    def next(self):
        if(auto_globals.enabled and auto_globals.good_fix and auto_globals.good_imu and not auto_globals.fix_timeout):
            return self._stateMachine.turnTowardWaypoint

        return self._stateMachine.idle
