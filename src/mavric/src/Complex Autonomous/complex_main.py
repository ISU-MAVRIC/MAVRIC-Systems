#!/usr/bin/env python

#import modules
import complex_globals
import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from mavric.msg import Autonomous, Waypoint, Drivetrain, Steertrain, GPS, LED

# import pathing, and detection classes
from pathing import Pathing
import ArucoDetectionThread as aruco

# state machine class
class AutonomousStateMachine(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, AutonomousStateMachine.idle)


AutonomousStateMachine.idle = Idle(AutonomousStateMachine)
AutonomousStateMachine.nextPathPoint = NextPathPoint(AutonomousStateMachine)
AutonomousStateMachine.turnTowardPathPoint = TurnTowardPathPoint(AutonomousStateMachine)
AutonomousStateMachine.driveTowardPathPoint = DriveTowardPathPoint(AutonomousStateMachine)
AutonomousStateMachine.bankTowardPathPoint = BankTowardPathPoint(AutonomousStateMachine)
AutonomousStateMachine.reachedWaypoint = ReachedWayPoint(AutonomousStateMachine)

# define functions


def hms_to_s(h, m, s):
    # convert from hours-minutes-seconds to seconds
    return (h * 60 * 60) + (m * 60) + s


def cmd_cb(data):
    # enable/disable autonomous, forget current waypoints
    if data.command == 'E':
        g.enabled = True
        g.indicator_pub(True, 255, "RED")
        g.indicator_pub(False, 255, "BLUE")

    elif data.command == 'D':
        g.enabled = False
        g.indicator_pub(True, 255, "BLUE")
        g.indicator_pub(False, 255, "RED")

    elif data.command == 'F':
        g.waypoints = []


def waypoint_cb(data):
    # add new waypoint to list
    g.waypoints.append([data.latitude, data.longitude])
    g.waypoint_id.append(data.id)

def gps_fix_cb(data):
    g.good_fix = True #data.data


def gps_cb(data):
    # how often does the gps publish? do we have to compare values here?

    g.prev_position = g.position
    g.position = [data.longitude, data.latitude]
    g.fix_time = hms_to_s(data.time_h, data.time_m, data.time_s)
    #auto_globals.heading = data.heading


def imu_cb(data):
    if g.good_imu:
        g.heading = data.z


def imu_cal_cb(data):
    g.good_imu = True

    # if data.z > 0:
    #    auto_globals.good_imu = True
    # else:
    #    auto_globals.good_imu = False

# main loop
auto = AutonomousStateMachine()
g.path = Pathing(0.81, 1.20)


def main():
    # setup ROS node
    rospy.init_node("ANS")

    #init publishers
    auto_globals.drive_pub = rospy.Publisher("Drive_Train", Drivetrain, queue_size=10)
    auto_globals.steer_pub = rospy.Publisher("Steer_Train", Steertrain, queue_size=10)
    auto_globals.debug_pub = rospy.Publisher("Autonomous_Debug", String, queue_size=10)
    auto_globals.indicator_pub = rospy.Publisher("/indicators/light_pole", LED, queue_size=10)

    cmd_sub = rospy.Subscriber("Autonomous", Autonomous, cmd_cb, queue_size=10)
    way_sub = rospy.Subscriber("Next_Waypoint", Waypoint, waypoint_cb, queue_size=10)
    gps_fix_sub = rospy.Subscriber("GPS_Fix", Bool, gps_fix_cb, queue_size=10)
    gps_sub = rospy.Subscriber("GPS", GPS, gps_cb, queue_size=10)

    imu_sub = rospy.Subscriber("FusedAngle", Vector3, imu_cb, queue_size=10)
    imu_cal_sub = rospy.Subscriber("SensorCalibrations", Vector3, imu_cal_cb, queue_size=10)

    
    rate = rospy.Rate(2)  # 2 Hz

    while not rospy.is_shutdown():
        auto.run()
        g.path.run()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
