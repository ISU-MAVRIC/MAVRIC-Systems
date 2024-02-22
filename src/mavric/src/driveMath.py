#!/usr/bin/env python
import math


# Drive Math
# Jackson Milburn
# 2/19/2024


# The purpose of this script is to take in values from -100 to 100 for both the speed and direction from the controller.
# Using the dimensions of the wheelbase and track of the rover the script will return speed values from -100 to 100 representing the motor speeds of each wheel.
# The output of the direction of each wheel will be in degrees.
# The equations used to calculate both the speeds and directions were found from: https://www.mathworks.com/help/sm/ug/mars_rover.html


wheelbase = 42 # Wheelbase of the rover in inches
track = 26.5 # Track of the rover in inches
wheel_r = 5 # Radius of the wheel in inches

max_steer_angle = 45 # Max overall steering angle of the rover


# Initialzing speed and direction var for each wheel

FL_v = 0 # Front left angular velocity
FR_v = 0 # Front right angular velocity
ML_v = 0 # Mid left angular velocity
MR_v = 0 # Mid right angular velocity
BL_v = 0 # Back left angular velocity
BR_v = 0 # Back right angular velocity

FL_a = 0 # Front left angle
FR_a = 0 # Front right angle
BL_a = 0 # Back left angle
BR_a = 0 # Back right angle


def car_drive(drive, steer):

    # Checks for whether or not input from controller is within the 5% deadzone.

    if steer >= -5 and steer <= 5:

        # Initializing velocity and steering vars

        steer_angle = 0
        FR_a = 0
        FL_a = 0
        FL_v = drive
        FR_v = drive
        ML_v = drive
        MR_v = drive

    else:

        steer_angle = steer*math.radians(max_steer_angle)*0.01 # Converts -100 to 100 value into angle in radians
        R = (wheelbase/2)/math.tan(steer_angle) # Calculates the turn radius

        FR_a = math.degrees(math.atan((wheelbase/2)/(R-track/2))) # Calculates front right angle in degrees
        FL_a = math.degrees(math.atan((wheelbase/2)/(R+track/2))) # Calculates front left angle in degrees

        FL_v = (drive*math.sqrt((wheelbase/2)**2+(R+track/2)**2))/R # Calculates front left velocity from -100 to 100
        FR_v = (drive*math.sqrt((wheelbase/2)**2+(R-track/2)**2))/R # Calculates front right velocity from -100 to 100


        # Checks for proper sign for angles

        if steer < 0: 
            FL_v = -FL_v
            FR_v = -FR_v


        # Checks for proper velocity adjustment depenting on turn direction
            
        if abs(FL_v) > abs(FR_v) and abs(FL_v) > 100:
            v_adj = abs(FL_v*0.01)
        elif abs(FR_v) > abs(FL_v) and abs(FR_v) > 100:
            v_adj = abs(FR_v*0.01)
        else:
            v_adj = 1


        # Calculates the velocity of each wheel depending on the velocity adjustment and turn direction

        FL_v = FL_v/v_adj
        FR_v = FR_v/v_adj
        ML_v = (drive*(R+track/2))/R/v_adj
        MR_v = (drive*(R-track/2))/R/v_adj

    # Assigning correct magnitude and sign to the opposing wheel velocity and angle

    BL_a = -FL_a
    BR_a = -FR_a

    BL_v = FL_v
    BR_v = FR_v

    return [FL_v, ML_v, BL_v, FR_v, MR_v, BR_v, FL_a, BL_a, FR_a, BR_a]


def point_drive(drive,steer):

    outer_pd_R = math.sqrt((track/2)**2 + (wheelbase/2)**2) # Radius of the outer wheels
    inner_pd_R = track/2 # Radius of the inner wheels (middle)

    pd_angle = math.degrees(math.atan(outer_pd_R/inner_pd_R)) # Angle of the outer wheels in point drive

    v_adj = inner_pd_R/outer_pd_R # Ratio of the inner to outer radius


    # Adjusts the velocity and direction of each respective wheel

    FL_v = steer
    ML_v = steer*v_adj
    BL_v = steer
    FR_v = -steer
    MR_v = -steer*v_adj
    BR_v = -steer


    # Adjusts the correct direction of the outer wheel angles

    FL_a = pd_angle
    BL_a = -pd_angle
    FR_a = -pd_angle
    BR_a = pd_angle

    return [FL_v, ML_v, BL_v, FR_v, MR_v, BR_v, FL_a, BL_a, FR_a, BR_a]