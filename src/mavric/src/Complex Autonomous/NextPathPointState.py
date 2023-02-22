import complex_globals as g
from StateMachine import State

"""
Description:
- Used for setting the rover up to head towards then next path point
- Also decides which turning type to use to get to the next path point 
"""
class NextPathPoint(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine

    def enter(self):
        """
        Enters from:
        - Idle
        - ReachedWaypoint
        - TurnTowardPathPoint
        - DriveTowardPathPoint
        """
        g.drive_pub.publish(0, 0, 0, 0, 0, 0)
        g.steer_pub.publish(0, 0, 0, 0)

        g.pathpoint_num += 1
        g.debug_pub.publish("# of pathpoints, index")
        g.debug_pub.publish(str(len(g.pathpoints["linear"]))+ ", "+str(g.pathpoint_num))

    def run(self):
        """
        Description:
        - Checks how the rover should get to the next waypoint
        - If linear is true, the rover will point steer then drive to the next waypoint
        - If linear is false, the rover will just drive to the next waypoint
        """
        #ADVISOR NOTE: The linear boolean and bank toward path point were made for obstacle avoidance and isnt needed for normal autonomous functionality
        self.linear = g.pathpoints["linear"][g.pathpoint_num]
        g.debug_pub.publish("en, waypoints>0, gps, imu, gps timeout")
        g.debug_pub.publish("%d, %d, %d, %d, %d" % (g.enabled, len(g.waypoints) > 0, g.good_fix, g.good_imu, g.fix_timeout))
    
    def next(self):
        """
        Exits to:
        - Idle
        - TurnTowardPathPoint
        - BankTowardPathPoint
        """
        if not g.enabled or not g.good_fix or g.fix_timeout:
            return self._stateMachine.idle
        
        if self.linear is True:
            return self._stateMachine.turnTowardPathPoint
        
        if self.linear is False:
            return self._stateMachine.bankTowardPathPoint

        return self._stateMachine.nextPathPoint