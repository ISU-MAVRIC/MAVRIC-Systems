#!/usr/bin/env python
import math

wheel_length = 37.5
wheel_width = 28.5

sensdrive = 1
senssteer = 0.75

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

def radians(degrees):
    return degrees * math.pi / 180

def degrees(radians):
    return radians * (180 / math.pi)

def car_drive(drive,steer):
    in_angle = radians(abs(steer*100*senssteer))*0.9

    if not in_angle == 0:
        out_angle = math.pi/2 - math.atan(1/math.tan(in_angle) + 2*wheel_width/wheel_length)
        in_r = wheel_length / (2 * math.sin(in_angle))
        out_r = wheel_length / (2 * math.sin(out_angle))
        center_r = in_r * math.cos(in_angle) + wheel_width / 2
        in_mid_r = center_r - wheel_width / 2
        out_mid_r = center_r + wheel_width / 2
        out_v = -100*drive * sensdrive
        angle_v = out_v / out_r
        in_v = in_r * angle_v
        in_mid_v = in_mid_r * angle_v
        out_mid_v = out_mid_r * angle_v
    else:
        out_angle = 0
        in_v = 0
        in_mid_v = 0
        out_v = 0
        out_mid_v = 0
    if steer < 0:
        return [in_v, in_mid_v, in_v, out_v, out_mid_v, out_v, degrees(in_angle), degrees(in_angle), degrees(out_angle), degrees(out_angle)]
    elif steer > 0:
        return [out_v, out_mid_v, out_v, in_v, in_mid_v, in_v, -1*degrees(out_angle), -1*degrees(out_angle), -1*degrees(in_angle), -1*degrees(in_angle)]
    else:
        return [-drive*sensdrive*100, -drive*sensdrive*100, -drive*sensdrive*100, -drive*sensdrive*100, -drive*sensdrive*100, -drive*sensdrive*100, 0, 0, 0, 0]
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