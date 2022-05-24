from imutils.video import VideoStream
import imutils
import time
import cv2
import threading
import complex_globals as g


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()

def get_markers_from_frame(frame):
    markerLocations = []
    markerIds = []
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
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
    return (markerIds, markerLocations)



def aruco_detection():
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    vs = VideoStream(src=2).start()
    vs2 = VideoStream('rtsp://admin:mavric-camera@192.168.1.64:554/out.h264').start()
    time.sleep(1)
    while True:
        frame = vs.read()
        frame2 = vs2.read()
        time.sleep(.01)
        markers1 = get_markers_from_frame(frame)
        markers2 = get_markers_from_frame(frame2)
        camera_type = []
        pixel_location = []
        for index, ID in enumerate(markers1[0]):
            pixel_location.append(markers1[1][index])
            markers1[1][index] = (float(markers1[1][index][0] - 320) / 320) * 34.5
            camera_type.append('realsense')
        for index, ID in enumerate(markers2[0]):
            if ID not in markers1[0]:
                pixel_location.append(markers2[1][index])
                markers1[0].append(markers2[0][index])
                markers1[1].append((float(markers2[1][index][0] - 620) / 620) * 54.5) 
                camera_type.append('dome')
        g.posts["id"] = markers1[0]
        g.posts["heading"] = markers1[1]
        g.posts["type"] = camera_type
        g.posts["pixel_location"] = pixel_location


    vs.stop()
    vs2.stop()

thread = threading.Thread(target=aruco_detection)

def start_aruco_detection():
    thread.start()

start_aruco_detection()
