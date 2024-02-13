#!/usr/bin/env python
import math

# This code is gobaldy gook
# If anyone wants to change it, go right ahead. No one but Jacob P. knows how this code works.
# This code was imported from the old base station.

wheel_length = 42 #inches
wheel_width = 26.5 #inches
wheel_radius = 10 #inches

lf = 0
lm = 0
lb = 0
rf = 0
rm = 0
rb = 0
strLf = 0
strLb = 0
strRf = 0
strRb = 0

def car_drive(drive,steer):

    if drive <= 0:
        v = 
    elif drive :
        
    if steer_val != 0:
        steer_angle = 2*math.pi*(drive/100)
    else:
        steer_angle = 0
    
    if S_theta==0:
        FL_theta=0
        FR_theta=0
    else:
        R = (L/2)/math.tan(math.radians(S_theta))
        FL_theta = math.atan((L/2)/(R-w/2)) # Front left turn angle
        FR_theta = math.atan((L/2)/(R+w/2)) # Front right turn angle

    BL_theta = -FL_theta # Back left turn angle
    BR_theta = -FR_theta # Back right turn angle

    # Prevents divide by zero for R
    if S_theta==0:
        angular_v_FL = (v/wheel_r)
        angular_v_FR = (v/wheel_r)
        angular_v_ML = (v/wheel_r)
        angular_v_MR = (v/wheel_r)
    else:
        angular_v_FL = (v/wheel_r)*(math.sqrt((L/2)**2+(R-w/2)**2)/R)
        angular_v_FR = (v/wheel_r)*(math.sqrt((L/2)**2+(R+w/2)**2)/R)
        angular_v_ML = (v/wheel_r)*((R-w/2)/R)
        angular_v_MR = (v/wheel_r)*((R+w/2)/R)

    angular_v_BL = angular_v_FL
    angular_v_BR = angular_v_FR


def point_drive(drive,steer):
    str_angle = degrees(math.atan(wheel_length/wheel_width))
    str_r = math.sqrt(math.pow(wheel_width/2,2) + math.pow(wheel_length/2,2))
    mid_r = wheel_width/2
    str_v = abs(steer*sensdrive*100)
    mid_v = str_v*mid_r/str_r
    if steer < 0:
        return [-str_v, -mid_v, -str_v, str_v, mid_v, str_v, -str_angle, -str_angle, str_angle, str_angle]
    if steer > 0:
        return [str_v, mid_v, str_v, -str_v, -mid_v, -str_v, -str_angle, -str_angle, str_angle, str_angle]
    else:
        return [0, 0, 0, 0, 0, 0, -str_angle, -str_angle, str_angle, str_angle]
    return [0,0,0,0,0,0,0,0,0,0]
