#!/usr/bin/env python3

from ArucoClass import Aruco

aruco = Aruco(display=True)

loop = True
if __name__ == "__main__":
    try:
        while loop:
            frame = aruco.grab_frame()
            markers = aruco.get_markers(frame)
            angles = aruco.get_angles(markers)
            print(angles)
    except KeyboardInterrupt:
        loop = False

