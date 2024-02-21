from imutils.video import VideoStream
import imutils
import time
import cv2
import threading


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
    # vs2 = VideoStream(src=0).start()
    vs2 = VideoStream('rtsp://admin:mavric-camera@192.168.1.64:554/out.h264').start()
    time.sleep(1)
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
        print(angles)
        if cv2.waitKey(1) and 0xFF == ord('q'):
            break

    vs2.stop()

def start_aruco_detection():
    thread.start()

if __name__ == "__main__":
    try:
        thread = threading.Thread(target=aruco_detection)
        thread.start()
    except KeyboardInterrupt:
        thread._stop()

def aruco_steering(markers2):
    steer = []
    if angles >= -18:
       print(1)
    else: 
        print(-1)

# def aruco_speed(markerCorners):

