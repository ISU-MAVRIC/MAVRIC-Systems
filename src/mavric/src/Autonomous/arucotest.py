#!/usr/bin/env python3
import rospy
from mavric.msg import Drivetrain, Steertrain

from std_msgs.msg import Bool
from imutils.video import VideoStream
import imutils
import time
import cv2
import threading
import numpy as py
from driver import Driver

driveSpeed = 7
lTheta = -45
rTheta = 12
driveMath = Driver()
enable = False

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

def get_markers_from_frame(frame):
    markerLocations = []
    markerIds = []
    markerCorners= []
    (corners, ids, rejected) = detector.detectMarkers(frame)
    if (len(corners) > 0):
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            markerLocations.append((cX, cY))
            markerIds.append(markerID)
            markerCorners.append((topLeft, bottomRight))
    return (markerIds, markerLocations, markerCorners)


def aruco_detection():
    global drive_pub, steer_pub, enable
    # vs2 = VideoStream(src=0).start()
    vs2 = VideoStream('rtsp://admin:mavric-camera@192.168.1.64:554/out.h264').start()
    time.sleep(1)
    lastFix = 0
    currentFix = 5
    drive = 0
    steer = 0
    fault = 0
    while True:
        frame2 = vs2.read()
        time.sleep(.01)
        markers2 = get_markers_from_frame(frame2)
        angles = []
        for index, ID in enumerate(markers2[0]):
            angles.append((float(markers2[1][index][0] - 973) / 973) * 48.9 )
            frame2 = cv2.rectangle(frame2, markers2[2][index][0], markers2[2][index][1], (255, 0, 0), 2)
            frame2 = cv2.rectangle(frame2, markers2[2][index][0], (markers2[2][index][0][0]+100, markers2[2][index][0][1]-20), (0, 0, 0), -1)
            frame2 = cv2.putText(frame2, "ID: " + str(ID) + " Theta: " + str(int(angles[index])), markers2[2][index][0], cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 255, 255), 1, cv2.LINE_AA)
        
        # cv2.imshow('Frame',frame2)
        if len(angles) > 0:
            if fault <= 0:
                lastFix = time.perf_counter()
                fault = 0
                steer = 100/(rTheta-lTheta) * (angles[0] - (rTheta+lTheta)/2)
                drive = driveSpeed
            fault = fault - 1
        else:
            currentFix = time.perf_counter()
            fault = fault + 1
            if fault >= 5:
                if currentFix - lastFix > 3:
                    steer = 0
                    drive = 0
                elif currentFix - lastFix > 1:
                    steer = 0
                fault = 5
        if enable:
            lf, lm, lb, rf, rm, rb, lfs, lbs, rfs, rbs = driveMath.v_car_steer(drive, steer)
            drive_pub.publish(lf,lm,lb,rf,rm,rb)
            steer_pub.publish(lfs,lbs,rfs,rbs)
        else:
            drive_pub.publish(0,0,0,0,0,0)
            steer_pub.publish(0,0,0,0)
        if cv2.waitKey(1) and 0xFF == ord('q'):
            break

    vs2.stop()

def start_aruco_detection():
    thread.start()

def enable_cb(data):
    global enable
    if data.data:
        enable = True
    else:
        enable = False

if __name__ == "__main__":
    try:
        rospy.init_node("Aruco")
        drive_pub = rospy.Publisher("/Drive/Drive_Command", Drivetrain, queue_size=10)
        steer_pub = rospy.Publisher("/Drive/Steer_Command", Steertrain, queue_size=10)
        enable_sub = rospy.Subscriber("/Auto/Enable", Bool, enable_cb)
        thread = threading.Thread(target=aruco_detection)
        thread.start()
    except rospy.ROSInterruptException:
        thread._stop()


    


