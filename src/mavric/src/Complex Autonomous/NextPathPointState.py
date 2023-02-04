import complex_globals as g
from StateMachine import State


class NextPathPoint(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine

    def enter(self):
        g.drive_pub.publish(0, 0, 0, 0, 0, 0)
        g.steer_pub.publish(0, 0, 0, 0)

        g.pathpoint_num += 1
        g.debug_pub.publish("# of pathpoints, index")
        g.debug_pub.publish(str(len(g.pathpoints["linear"]))+ ", "+str(g.pathpoint_num))
        g.desired_heading = 0
        g.state = "NextPathPoint"

    def run(self):
        self.linear = g.pathpoints["linear"][g.pathpoint_num]
        g.debug_pub.publish("en, waypoints>0, gps, imu, gps timeout")
        g.debug_pub.publish("%d, %d, %d, %d, %d" % (g.enabled, len(g.waypoints) > 0, g.good_fix, g.good_imu, g.fix_timeout))
    
    def next(self):
        if not g.enabled or not g.good_fix or g.fix_timeout:
            return self._stateMachine.idle
        
        if self.linear is True:
            return self._stateMachine.turnTowardPathPoint
        
        if self.linear is False:
            return self._stateMachine.bankTowardPathPoint

        return self._stateMachine.nextPathPoint