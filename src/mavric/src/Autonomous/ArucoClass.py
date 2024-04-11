from imutils.video import VideoStream
import cv2
import time


class Aruco():
    def __init__(self,display=False):
        self.print = display
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary,self.parameters)
        #self.vs = VideoStream('rtsp://admin:mavric-camera@192.168.1.64:554/out.h264').start()
        self.vs = VideoStream(src=0).start()
        self.idtimes = [0,0,0,0,0,0,0,0,0,0]

    def aruco_detection(self):
        self.frame = self.vs.read()
        markers = self.get_markers(self.frame)
        angles = []
        for index, ID in enumerate(markers[0]):
            angles.append((float(markers[1][index][0] - 973) / 973) * 48.9 )
            if self.print:
                self.frame = cv2.rectangle(self.frame, markers[2][index][0], markers[2][index][1], (255, 0, 0), 2)
                self.frame = cv2.rectangle(self.frame, markers[2][index][0], (markers[2][index][0][0]+100, markers[2][index][0][1]-20), (0, 0, 0), -1)
                self.frame = cv2.putText(self.frame, "ID: " + str(ID) + " Theta: " + str(int(angles[index])), markers[2][index][0], cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("Frame",self.frame)
                cv2.waitKey(1)
        return(angles)
    
    def grab_frame(self):
        return self.vs.read()
    
    def showframe(self):
        frame = self.vs.read()
        cv2.imshow("Frame",frame)
        cv2.waitKey(1)

    def get_markers(self, frame):
        markerLocations = []
        markerIds = []
        markerCorners= []
        (corners, ids, rejected) = self.detector.detectMarkers(frame)
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
    
    def __del__(self):
        self.vs.stop()