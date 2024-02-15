#!/usr/bin/env python
import math

# This code is gobaldy gook
# If anyone wants to change it, go right ahead. No one but Jacob P. knows how this code works.
# This code was imported from the old base station.

wheelbase = 42 #inches
track = 26.5 #inches
wheel_r = 5 #inches

max_steer_angle = 45 # degrees

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

    if steer >= -5 and steer <= 5:
        steer_angle = 0
        FR_a = 0
        FL_a = 0
        FL_v = drive
        FR_v = drive
        ML_v = drive
        MR_v = drive
    else:
        steer_angle = steer*math.radians(max_steer_angle)*0.01
        R = (wheelbase/2)/math.tan(steer_angle)
        FR_a = math.degrees(math.atan((wheelbase/2)/(R-track/2))) # Front right turn angle
        FL_a = math.degrees(math.atan((wheelbase/2)/(R+track/2))) # Front left turn angle

        FL_v = (drive*math.sqrt((wheelbase/2)**2+(R+track/2)**2))/R
        FR_v = (drive*math.sqrt((wheelbase/2)**2+(R-track/2)**2))/R

        if steer < 0:
            FL_v = -FL_v
            FR_v = -FR_v

        v_adj = 1

        if abs(FL_v) > abs(FR_v) and abs(FL_v) > 100:
            v_adj = abs(FL_v*0.01)
        elif abs(FR_v) > abs(FL_v) and abs(FR_v) > 100:
            v_adj = abs(FR_v*0.01)

        FL_v = FL_v/v_adj
        FR_v = FR_v/v_adj
        ML_v = (drive*(R+track/2))/R/v_adj
        MR_v = (drive*(R-track/2))/R/v_adj

    BL_a = -FL_a # Back left turn angle
    BR_a = -FR_a # Back right turn angle

    BL_v = FL_v
    BR_v = FR_v

    return [FL_v, ML_v, BL_v, FR_v, MR_v, BR_v, FL_a, BL_a, FR_a, BR_a]

print(car_drive(50, 25))

'''
def point_drive(drive,steer):
    steer_angle = steer*math.radians(max_steer_angle)*0.01
    R = math.sqrt((track/2)**2 + (wheelbase/2)**2)

    FL_v = drive
    ML_v = drive
    BL_v = drive
    FR_v = drive
    MR_v = drive
    BR_v = drive

    mid_v = (str_v*(track/2))/R
    if steer < 0:
        return [FL_v, ML_v, BL_v, FR_v, MR_v, BR_v, FL_a, BL_a, FR_a, BR_a]
    if steer > 0:
        return [FL_v, ML_v, BL_v, FR_v, MR_v, BR_v, FL_a, BL_a, FR_a, BR_a]
    else:
        return [0, 0, 0, 0, 0, 0, -str_angle, -str_angle, str_angle, str_angle]
    return [0,0,0,0,0,0,0,0,0,0]

print(car_drive(50, 25))
'''