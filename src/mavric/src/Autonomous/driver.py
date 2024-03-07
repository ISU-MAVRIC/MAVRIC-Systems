#!/usr/bin/env python3
"""
Library for calulating the drive motor speeds and steer motor positions when using car and point steering

Author: Nathan Logston, Jacob Peskuski
"""
import math

"""
Motor speed and postions calculator module, group of functions for calulating speeds and positions for car steer and point steer
Input parameters:
	wheelLength = distance between front and back wheels
	wheelWidth = distance between left and right wheels
"""


class Driver():
    def __init__(self, wheelLength=42, wheelWidth=26.5, threshold=25):
        self.L = wheelLength
        self.W = wheelWidth
        self.threshold = threshold
        self.wheelR = 5 # Radius of wheel in inches

        # Geometric maximum steering angle (when turn radius is equal to half the wheel width)
        # The true value would result in a divide by zero error for the inner wheel angle, so we go 10 degrees lower than true value to prevent this
        # This lowering value is arbitrary. If you feel like the rover is cranking turns like an F1 car, increase this modification angle to make the max lower. 
        self.max_steer_angle = math.atan(self.L/self.W) - math.radians(10)


        # --- Values for Point Steer ---
        # point steer motor position (deg)
        self.pointAngle = math.degrees(math.atan(self.L/self.W))
        # point steer radiuses (untits of w and l)
        self.str_r = math.sqrt(math.pow(self.W/2, 2)+math.pow(self.L/2, 2))
        self.mid_r = self.W/2

    """
    CAR DRIVING
	Function for calulating car steering speeds and postitions from rover velocity
	Input paramters:
		drive = speed of wheels on the outside of the turning circle (-100<-->100)
		steer = steer percentage of maximum steer angle (-100<-->100)
	Output parameters:
		left front drive, left middle drive, left back drive, right front drive, right middle drive, right back drive,
		left front steer, left back steer, right front steer, right back steer
	"""

    def v_car_steer(self, drive, steer):
        if drive < self.threshold and drive != 0:  # check if drive throttle is less than threshold and greater than zero
            drive = math.copysign(self.threshold, drive)    #if less, set to threshold with same sign as input
        if not steer == 0:  # if the steer is actually trying to turn (prevents divide by zero)
            steer_angle = abs(steer)*self.max_steer_angle*0.01 # convert steering command (-100 to 100) into a steering angle

            R = (self.L/2)/math.tan(steer_angle)    # Turn Radius (rad)
            in_mid_radius = R - self.W/2    # radius from turn "center" to inner mid wheel
            out_mid_radius = R + self.W/2   # radius from turn "center" to outer mid wheel
            out_angle = math.atan(self.L/2 / (out_mid_radius))  # angle that the outer wheels must turn (rad)
            in_angle = math.atan(self.L/2 / (in_mid_radius))    # angle that the inner wheels must turn (rad)

            out_radius = math.sqrt(math.pow(self.L/2, 2) + math.pow(out_mid_radius, 2)) # radius of outer wheels from turn center
            in_radius = math.sqrt(math.pow(self.L/2, 2) + math.pow(in_mid_radius, 2))   # radius of inner wheels from turn center

            # the fastest wheel is the wheel on the outside corner due to largest radius
            # so each wheel speed is scaled according to that
            # this is based on V = R*omega, where omega is the rotational speed of the rover around the turn center.
            # Since omega is constant for each wheel, each wheel can be scaled from the fastest wheel with V_fast/R_fast = V_slower/R_slower
            out_v = drive
            out_mid_v = drive * out_mid_radius / out_radius
            in_mid_v = drive * in_mid_radius / out_radius
            in_v = drive * in_radius / out_radius

        if steer < 0:   # turning left
            return in_v, in_mid_v, in_v, out_v, out_mid_v, out_v, -math.degrees(in_angle), math.degrees(in_angle), \
                -math.degrees(out_angle), math.degrees(out_angle)
        elif steer > 0: # turning right
            return out_v, out_mid_v, out_v, in_v, in_mid_v, in_v, math.degrees(out_angle), -math.degrees(out_angle), \
                math.degrees(in_angle), -math.degrees(in_angle)
        else:   # going straight
            return drive, drive, drive, drive, drive, drive, 0, 0, 0, 0

    """
    POINT STEER
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
        return str_v, mid_v, str_v, -str_v, -mid_v, -str_v, self.pointAngle, -self.pointAngle, -self.pointAngle, self.pointAngle
