#!/usr/bin/env python3

"""
Rover: SCARAB
Status: Experimental
Description:
Main script for the rovers autonomous system. Uses sensor data to control the rover to travel to different waypoints. 
Consists of 3 parts:
- Aruco detection: Uses cameras to search for aruco tags
- Pathing: Creates a path for the rover to travel, using pathpoints and waypoints
- State Machine: Controls the rover, what the rover does is based on the state being used at the time
"""

#import globals file, holds variable data used by arucvo detection, pathing, and all the states
# if you want variable's data to be shared across the autonomous system, it will need to be put in the complex_globals.py
import complex_globals as g

# import modules from ROS
import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from mavric.msg import Autonomous, Waypoint, Drivetrain, Steertrain, GPS, LED

# import pathing, and detection classes, and all states.
from pathing import Pathing

#ADVISOR NOTE: Currently importing the aruco detection thread starts when it is imported, I recomend turning this into a class and starting it in the main function
import ArucoDetectionThread as aruco

from StateMachine import StateMachine, State
from IdleState import Idle
from NextPathPointState import NextPathPoint
from TurnTowardPathPointState import TurnTowardPathPoint
from DriveTowardPathPointState import DriveTowardPathPoint
from BankTowardPathPointState import BankTowardPathPoint
from ReachedWaypointState import ReachedWayPoint

# state machine class, uses the statemachine template object from StateMachine.py
# when this class is created, it defualts to the idle state
class AutonomousStateMachine(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, AutonomousStateMachine.idle)

# load all the states into the state machine object as parameters, this is done so each state has access to all the other states
# all states use the state template object in StateMachine.py
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

# define subscriber call backs

def cmd_cb(data):
    """
    Topic: Autonomous
    Msg Type: Autonomous
    Description: Autonomous command data from base station. Consists of a single charcter which means:
    - E: Enable autonomous
    - D: Disable autonomous
    - F: Clear waypoints
    """
    # enable/disable autonomous, forget current waypoints
    if data.command == 'E':
        g.enabled = True
        g.indicator_pub.publish(True, 255, "RED")
        g.indicator_pub.publish(False, 255, "BLUE")

    elif data.command == 'D':
        g.enabled = False
        g.indicator_pub.publish(True, 255, "BLUE")
        g.indicator_pub.publish(False, 255, "RED")

    elif data.command == 'F':
        g.waypoints = []


def waypoint_cb(data):
    """
    Topic: Next_Waypoint
    Msg Type: Waypoint
    Description: Waypoint command with lon and lat data from basse station
    """

    # add new waypoint to list
    g.waypoints.append([data.longitude, data.latitude])
    g.waypoint_id.append(data.id)


def gps_fix_cb(data):
    """
    Topic: GPS_Fix
    Msg Type: Bool
    Description: Boolean for if the GPS has a sattelite fix and can determine position or not. The state machine should be idle if there is no GPS fix.
    """
    g.good_fix = data.data 

#ADVISOR NOTE: While heading and velocity are not used, they might more accurate then the IMU when the rover is traveling to a waypoint. If GPS velocity and heading are going to be used, they should have their own variables.
def gps_cb(data):
    """
    Topic: GPS
    Msg Type: GPS
    Description: Data from the GPS. Updates around once every second. Contains lon, lat, alt, heading, and velocity data
    """

    g.prev_position = g.position
    g.position = [data.longitude, data.latitude]
    g.fix_time = hms_to_s(data.time_h, data.time_m, data.time_s)
    #auto_globals.heading = data.heading


def imu_cb(data):
    """
    Topic: FusedAngle
    Msg Type: Vector3
    Description: Data from the IMU magnometer (basically a compass). Has x, y, z data, but only z is needed.
    """

    if g.good_imu:
        g.heading = data.z

#ADVISOR NOTE: Needs to be reworked if the IMU code is improved
def imu_cal_cb(data):
    """
    Topic: SensorCalibrations
    Msg Type: Vector3
    Description: Calibration data from IMU, used to check if the IMU is calibrated.
    """
    g.good_imu = True

    # if data.z > 0:
    #    auto_globals.good_imu = True
    # else:
    #    auto_globals.good_imu = False

# main code and autonomous/pathing init
auto = AutonomousStateMachine()
g.path = Pathing(0.81, 1.20)


def main():
    # setup ROS node
    rospy.init_node("ANS")

    #init publishers and subscribers
    g.drive_pub = rospy.Publisher("Drive_Train", Drivetrain, queue_size=10)
    g.steer_pub = rospy.Publisher("Steer_Train", Steertrain, queue_size=10)
    g.debug_pub = rospy.Publisher("Autonomous_Debug", String, queue_size=10)
    g.indicator_pub = rospy.Publisher("/indicators/light_pole", LED, queue_size=10)

    cmd_sub = rospy.Subscriber("Autonomous", Autonomous, cmd_cb, queue_size=10)
    way_sub = rospy.Subscriber("Next_Waypoint", Waypoint, waypoint_cb, queue_size=10)
    gps_fix_sub = rospy.Subscriber("GPS_Fix", Bool, gps_fix_cb, queue_size=10)
    gps_sub = rospy.Subscriber("GPS", GPS, gps_cb, queue_size=10)

    imu_sub = rospy.Subscriber("FusedAngle", Vector3, imu_cb, queue_size=10)
    imu_cal_sub = rospy.Subscriber("SensorCalibrations", Vector3, imu_cal_cb, queue_size=10)

    
    rate = rospy.Rate(2)  # 2 Hz

    # autonomous loop, while the aruco detection runs in its own loop this loop runs the pathing code and state machine
    while not rospy.is_shutdown():
        if len(g.waypoints) != 0:
            #g.debug_pub.publish("Running Pathing")
            g.path.run()
        auto.run()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
