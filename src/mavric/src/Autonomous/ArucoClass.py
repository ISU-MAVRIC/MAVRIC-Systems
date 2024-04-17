from imutils.video import VideoStream
import cv2
import time
import numpy as np

class Aruco():
    def __init__(self,display=True):
        self.print = display
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary,self.parameters)
        self.vs = VideoStream('rtsp://admin:mavric-camera@192.168.1.64:554/out.h264').start()
        self.mtx = np.array([[783.2520337429364,0.0,639.6473958102106],
                    [0.0,585.2162351288066,371.20009620432825],
                    [0.0,0.0,1.0]])
        self.dist = np.array([[-0.35652612625179947],
                     [0.12085730057198203],
                     [0.0018833852883372265],
                     [0.0018833852883372265],
                     [0.0018833852883372265],
                     [0.0018833852883372265],
                     [0.0017426867363701422],
                     [-0.017489950007839372]])
        # self.vs = VideoStream(src=0).start()
        self.markerSize = .15 # meters
        # marker point location (center) for solvePNP
        self.marker_points = np.array([[-self.markerSize / 2, self.markerSize / 2, 0],
                              [self.markerSize / 2, self.markerSize / 2, 0],
                              [self.markerSize / 2, -self.markerSize / 2, 0],
                              [-self.markerSize / 2, -self.markerSize / 2, 0]], dtype=np.float32)
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
    
    def get_dist(self, frame):
        '''
        Returns the distance from the tag in meters'''
        (corners, ids, rejected) = self.detector.detectMarkers(frame)
        rvecs = []
        tvecs = []
        trash = []
        for c in corners:
            nada, R, t = cv2.solvePnP(self.marker_points, c, self.mtx, self.dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        if len(tvecs) > 0:
            return tvecs[0][2]
        else:
            return tvecs
    
    def get_angles(self, markers):
        '''
        Returns the angle from -45 (left) to 45 (right) given the markers from a frame
        '''
        angles = []
        for index, ID in enumerate(markers[0]):
            '''
            this math takes the marker x coordinate and centers it, ie (position - frameWidth/2)
            so a center value at half the frame's width would return 0
            The next part maps it from pixels to arbitrary degrees. "90(deg) / width (pixels)" conversion factor
            We don't actually have the real angle to the rover, it's just an estimate anyways
            '''
            angles.append(90/self.width * (markers[1][index][0] - self.width/2))
        return angles


    def __del__(self):
        self.vs.stop()