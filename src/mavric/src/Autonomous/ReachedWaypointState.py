import auto_globals

from StateMachine import State

class ReachedWaypoint(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
    
    def enter(self):
        auto_globals.drive_pub.publish(0, 0)

    def run(self):
        pass

    def next(self):
	#eventually go into CV state

        if(len(auto_globals.waypoints) > 1):
            auto_globals.waypoints.pop(0)
	    return self._stateMachine.turnTowardWaypoint

	elif(len(auto_globals.waypoints) > 0):
	    auto_globals.waypoints.pop(0)

        return self._stateMachine.idle
