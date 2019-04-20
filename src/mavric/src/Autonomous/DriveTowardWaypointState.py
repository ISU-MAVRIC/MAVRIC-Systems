from StateMachine import State

class DriveTowardWaypoint(State):
    def __init__(self, stateMachine):
        self._stateMachine = stateMachine

    def enter(self):
        self.linear_error = auto_globals.LIN_ERROR_THRESHOLD * 2

    def run(self):
        auto_globals.prev_fix_time = auto_globals.fix_time

        #set first waypoint in array as target
        tgt = [0, 0]
        tgt[0] = auto_globals.waypoints[0][0]
        tgt[1] = auto_globals.waypoints[0][1]

        #capture position in case it changes later
        pos = auto_globals.position

        #solve the geodesic problem corresponding to these lat-lon values
        #   assumes WGS-84 ellipsoid model
        geod = Geodesic.WGS84.Inverse(pos[0], pos[1], tgt[0], tgt[1])

        #get linear error in meters
        linear_error = geod['s12']

        #calculate angle between pos and tgt
        #   assumes great-circle (spherical Earth) model

        #convert lat-lon to radians
        #pos = [radians(pos[0]), radians(pos[1])]
        #tgt = [radians(tgt[0]), radians(tgt[1])]

        #calculate heading to target, then angular error
        #kY = cos(tgt[0]) * sin(tgt[1] - pos[1])
        #kX = (cos(pos[0]) * sin(tgt[0])) - (sin(pos[0]) * cos(tgt[0]) * cos(tgt[1] - pos[1]))

        #tgt_head = degrees(atan2(kY, kX))
        #tgt_head = (tgt_head + 360) % 360
        #angular_error = tgt_head - heading


        #just use the 'azi1' parameter from the geodesic solution
        self.angular_error = geod['azi1'] - auto_globals.heading

        #apply power based on heading
        left_power = 0
        right_power = 0

        if(self.linear_error > auto_globals.LIN_ERROR_THRESHOLD):
            #drive forward
            left_power = auto_globals.Scale
            right_power = auto_globals.Scale

        auto_globals.drive_pub.publish(left_power * 100, right_power * 100)

        #remember linear error for the next cycle
        auto_globals.prev_linear_error = self.linear_error

    def next(self):
        if(not auto_globals.enabled or not auto_globals.good_fix or auto_globals.fix_timeout):
            return self._stateMachine.idle
            
        elif(self.linear_error <= auto_globals.LIN_ERROR_THRESHOLD):
            return self._stateMachine.reachedWaypoint

        return self._stateMachine.driveTowardWaypoint
