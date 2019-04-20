#!/usr/bin/env python

#import modules
import auto_globals
import rospy

from std_msgs.msg import String
from geometric_msgs.msg import Vector3
from mavric.msg import Autonomous, Waypoint, Drivetrain, GPS

from StateMachine import StateMachine, State
from IdleState import Idle
from TurnTowardWaypointState import TurnTowardWaypoint
from DriveTowardWaypointState import DriveTowardWaypoint
from ReachedWaypointState import ReachedWaypoint

#define state machine class
class Autonomous(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, Autonomous.idle)

Autonomous.idle = Idle(Autonomous)
Autonomous.turnTowardWaypoint = TurnTowardWaypoint(Autonomous)
Autonomous.driveTowardWaypoint = DriveTowardWaypoint(Autonomous)
Autonomous.reachedWaypoint = ReachedWaypoint(Autonomous)

#define functions
def hms_to_s(h,m,s):
    #convert from hours-minutes-seconds to seconds
    return (h * 60 * 60) + (m * 60) + s;

def cmd_cb(data):
    #enable/disable autonomous, forget current waypoints
    if data.command == 'E':
        auto_globals.enabled = True

    elif data.command == 'D':
        auto_globals.enabled = False

    elif data.command == 'F':
        auto_globals.waypoints = []

def waypoint_cb(data):
    #add new waypoint to list
    auto_globals.waypoints.append([data.latitude, data.longitude])

def gps_cb(data):
    #how often does the gps publish? do we have to compare values here?
    
    auto_globals.prev_position = auto_globals.position
    auto_globals.position = [data.latitude, data.longitude]
    auto_globals.fix_time = hms_to_s(data.time_h, data.time_m, data.time_s)
    auto_globals.good_fix = data.good_fix
    #auto_globals.heading = data.heading

def imu_cb(data):
    if auto_globals.good_imu:
        auto_globals.heading = data.z

def imu_cal_cb(data):
    auto_globals.good_imu = True
    
    #if data.z > 0:
    #    auto_globals.good_imu = True
    #else:
    #    auto_globals.good_imu = False

#main loop
auto = Autonomous()

def main():
    #setup ROS node
    rospy.init_node("ANS")

    #globalize publishers somehow
    auto_globals.drive_pub = rospy.Publisher("Drive_Train", Drivetrain, queue_size=10)
    auto_globals.debug_pub = rospy.Publisher("Autonomous_Debug", String, queue_size=10)

    cmd_sub = rospy.Subscriber("Autonomous", Autonomous, cmd_cb, queue_size=10)
    way_sub = rospy.Subscriber("Next_Waypoint", Waypoint, waypoint_cb, queue_size=10)
    gps_sub = rospy.Subscriber("/GPS_Data", GPS, gps_cb, queue_size=10)

    imu_sub = rospy.Subscriber("/Drive_Board_HW/IMU/FusedAngle", Vector3, imu_cb, queue_size=10)
    imu_cal_sub = rospy.Subscriber("/Drive_Board_HW/IMU/SensorCalibrations", Vector3, imu_cal_cb, queue_size=10)

    auto_globals.Scale = rospy.get_param("~Range", 0.5)
    auto_globals.Rover_Width = rospy.get_param("~Rover_Width", 1)
    auto_globals.Rover_MinTurnRadius = rospy.get_param("~Rover_MinTurnRadius", 2)

    rate = rospy.Rate(2)    #2 Hz
    
    while not rospy.is_shutdown():
        auto.run()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except:
        pass
