#!/usr/bin/env python3
import time
from ArucoClass import Aruco

aruco = Aruco(display=True)

loop = True
if __name__ == "__main__":
    try:
        while loop:
            frame = aruco.grab_frame()
            dist = aruco.get_dist(frame)
            markers = aruco.get_markers(frame)
            angles = aruco.get_angles(markers)
            print(dist, angles)
            aruco.showframe(frame)
    except KeyboardInterrupt:
        loop = False

