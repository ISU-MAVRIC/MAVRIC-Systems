#!/usr/bin/env python3

from ArucoClass import Aruco

aruco = Aruco(display=True)

loop = True
if __name__ == "__main__":
    try:
        while loop:
            frame = aruco.grab_frame()
            markers = aruco.get_markers(frame)
            print(markers)
            print(len(markers[0]))
            aruco.showframe()
    except KeyboardInterrupt:
        loop = False

