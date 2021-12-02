#!/usr/bin/env python
"""
Library for calulating the drive motor speeds and steer motor positions when using car and point steering

Author: Jacob Peskuski
"""
import math

"""
Motor speed and postions calculator module, group of functions for calulating speeds and positions for car steer and point steer
Input parameters:
	wheelLength = distance between front and back wheels
	wheelWidth = distance between left and right wheels
"""


class Driver():
    def __init__(self, wheelLength=37.5, wheelWidth=28.4, threshold=25):
        self.L = wheelLength
        self.W = wheelWidth
        self.threshold = threshold
        # point steer motor position (deg)
        self.pointAngle = math.degrees(math.atan(self.L/self.W))
        # point steer radiuses (untits of w and l)
        self.str_r = math.sqrt(math.pow(self.W/2, 2)+math.pow(self.L/2, 2))
        self.mid_r = self.W/2

    """
	Function for calulating car steering speeds and postitions from rover velocity
	Input paramters:
		drive = speed of wheels on the outside of the turning circle (-100<-->100)
		steer = angular position of the steer motors on the inside of the turn circle (-90<-->90)
	Output parameters:
		left front drive, left middle drive, left back drive, right front drive, right middle drive, right back drive,
		left front steer, left back steer, right front steer, right back steer
	"""

    def v_car_steer(self, drive, steer):
        if drive < self.threshold:
            drive = math.copysign(self.threshold, drive)
        in_angle = math.radians(abs(steer))*0.9
        if in_angle != 0:
            out_angle = math.pi/2 - math.atan(1/math.tan(in_angle)+2*self.W/self.L)
            in_r = self.L / (2 * math.sin(in_angle))
            out_r = self.L / (2 * math.sin(out_angle))
            center_r = in_r * math.cos(in_angle) + self.W / 2
            in_mid_r = center_r - self.W / 2
            out_mid_r = center_r + self.W / 2
            out_v = drive
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
            return in_v, in_mid_v, in_v, out_v, out_mid_v, out_v, math.degrees(in_angle), math.degrees(in_angle), \
                math.degrees(out_angle), math.degrees(out_angle)
        elif steer > 0:
            return out_v, out_mid_v, out_v, in_v, in_mid_v, in_v, -math.degrees(out_angle), -math.degrees(out_angle), \
                -math.degrees(in_angle), -math.degrees(in_angle)
        else:
            return drive, drive, drive, drive, drive, drive, 0, 0, 0, 0

    """
	Function for calulating car steering speeds and postitions from rover turn radius
	Input paramters:
		drive = speed of wheels on the outside of the turning circle (-100<-->100)
		radius = turning radius measured from center of circle (same units as wheel length and width) (make negative if turning left)
	Output parameters:
		left front drive, left middle drive, left back drive, right front drive, right middle drive, right back drive,
		left front steer, left back steer, right front steer, right back steer
	"""

    def r_car_steer(self, drive, radius):
        center_r = abs(radius)
        if center_r != 0:
            in_angle = math.pi/2 - \
                math.atan((2*center_r-self.W)/self.L)
            out_angle = math.pi/2 - \
                math.atan(1/math.tan(in_angle)+2*self.W/self.L)
            in_r = self.L / (2 * math.sin(in_angle))
            out_r = self.L / (2 * math.sin(out_angle))
            center_r = in_r * math.cos(in_angle) + self.W / 2
            in_mid_r = center_r - self.W / 2
            out_mid_r = center_r + self.W / 2
            out_v = drive
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

        if center_r < 0:
            return in_v, in_mid_v, in_v, out_v, out_mid_v, out_v, math.degrees(in_angle), math.degrees(in_angle), \
                math.degrees(out_angle), math.degrees(out_angle)
        elif center_r > 0:
            return out_v, out_mid_v, out_v, in_v, in_mid_v, in_v, -math.degrees(out_angle), -math.degrees(out_angle), \
                -math.degrees(in_angle), -math.degrees(in_angle)
        else:
            return drive, drive, drive, drive, drive, drive, 0, 0, 0, 0

    """
	Function for calulating point steering speeds and postitions 
	Input paramters:
		drive = speed of wheels on the outside of the turning circle (-100<-->100)
	Output parameters:
		left front drive, left middle drive, left back drive, right front drive, right middle drive, right back drive,
		left front steer, left back steer, right front steer, right back steer
	"""

    def v_point_steer(self, drive):
        if abs(drive) < self.threshold:
            drive = math.copysign(self.threshold, drive)
        str_v = drive
        mid_v = str_v*self.mid_r/self.str_r
        return str_v, mid_v, str_v, -str_v, -mid_v, -str_v, -self.pointAngle, -self.pointAngle, self.pointAngle, self.pointAngle
