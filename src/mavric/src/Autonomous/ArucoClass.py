from imutils.video import VideoStream
import cv2
import time
import numpy as py

class Aruco():
    def __init__(self,display=True):
        self.print = display
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary,self.parameters)
        #self.vs = VideoStream('rtsp://admin:mavric-camera@192.168.1.64:554/out.h264').start()
        self.vs = VideoStream(src=0).start()
        self.frame = self.vs.read()
        self.height, self.width = self.frame.shape[:2]

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
        '''
        Returns new frame data from the self.vs source
        '''
        return self.vs.read()
    
    def showframe(self, frame):
        '''
        Shows the input frame in a window
        '''
        cv2.imshow("Frame",frame)
        cv2.waitKey(1)

    def get_markers(self, frame):
        '''
        Returns the marker information, such as ID, center coordinate, and corner coordinates
        '''
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
    
    def get_dist(self, markers):
        '''
        Returns the calculated distance from the tag assuming that it's about 250 mm across.
        '''
        data = markers[0]
        vector1 = (py.sqrt((markers[2][0][0])**2 - (markers[2][0][1])**2))
        vector2 = (py.sqrt((markers[2][1][0])**2 - (markers[2][1][1])**2))
        distance = 1/abs(vector1-vector2)
        return distance
    
    def get_angles(self, markers):
        '''
        Returns the angle from -45 (left) to 45 (right) given the markers from a frame
        '''
        angles = []
        for index, ID in enumerate(markers[0]):
            '''
            mathwise this is a little bad, but really not that bad
            it takes the marker x coordinate and centers it, ie (position - frameWidth/2)
            so a center value at half the frame's width would return 0
            The next part maps it from pixels to arbitrary degrees.
            We don't acctually have the real angle to the rover, it's just an estimate anyways
            '''
            angles.append(90/self.width * (markers[1][index][0] - self.width/2))
        return angles


    def __del__(self):
        self.vs.stop()