import complex_globals as g
from StateMachine import State


class Idle(State):

    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
    

    def enter(self):
        g.drive_pub.publish(0, 0, 0, 0, 0, 0)
        g.steer_pub.publish(0, 0, 0, 0)


    def run(self):
        g.debug_pub.publish("en, waypoints>0, gps, imu, gps timeout")
        g.debug_pub.publish("%d, %d, %d, %d, %d" % (g.enabled, len(g.waypoints) > 0, g.good_fix, g.good_imu, g.fix_timeout))


    def next(self):
        #Checks for condition needed to turn
        if(g.enabled and len(g.waypoints) > 0 and g.good_fix and g.good_imu and not g.fix_timeout):
            # set next waypoint
            g.path.set_end_point(g.waypoints[0])
            return self._stateMachine.nextPathPoint

        return self._stateMachine.idle

