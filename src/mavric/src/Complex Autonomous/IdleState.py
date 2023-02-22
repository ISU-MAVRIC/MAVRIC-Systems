import complex_globals as g
from StateMachine import State

"""
Description:
- Passive state that the rover goes into when it has no waypoints or if sensor data is bad
- Rover does not move and waits for a waypoint and for all sensors to publish good data
"""
class Idle(State):

    def __init__(self, stateMachine):
        self._stateMachine = stateMachine
    

    def enter(self):
        """
        Enters from:
        - All states
        """
        g.drive_pub.publish(0, 0, 0, 0, 0, 0)
        g.steer_pub.publish(0, 0, 0, 0)


    def run(self):
        """
        Description:
        - Does nothing, only publishes debug messages
        """
        g.debug_pub.publish("en, waypoints>0, gps, imu, gps timeout")
        g.debug_pub.publish("%d, %d, %d, %d, %d" % (g.enabled, len(g.waypoints) > 0, g.good_fix, g.good_imu, g.fix_timeout))


    def next(self):
        """
        Exits to:
        - NextPathPoint
        """
        #Checks for condition needed to turn
        if(g.enabled and len(g.waypoints) > 0 and g.good_fix and g.good_imu and not g.fix_timeout):
            # set next waypoint
            g.path.set_end_point(g.waypoints[0])
            return self._stateMachine.nextPathPoint

        return self._stateMachine.idle

