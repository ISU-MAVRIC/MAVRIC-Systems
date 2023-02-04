#!/usr/bin/env python3

import pyrealsense2.pyrealsense2 as rs
import numpy as np
import math as m
import time
from threading import Thread

import cv2

import rospy
from mavric.msg import Aruco, Object, ObjectVector


y_fov = 0
x_fov = 0
x_res = 1
y_res = 1

d_threshold = 0
slope_threshold = 0

c_phi = 0
s_phi = 0
c_theta = 0
s_theta = 0


class Frames(Thread):
    global y_fov, x_fov, x_res, y_res, d_threshold, slope_threshold 

    def __init__(self):
        self.depth_frame = np.zeros((x_res, y_res))
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        config = rs.config()
        # Enable depth stream
        config.enable_stream(rs.stream.depth, x_res, y_res, rs.format.z16, 30)
        # Start streaming with chosen configuration
        self.pipe.start(config)
        self.running = True

        Thread.__init__(self)
    
    def get_depth_frame(self):
        return self.depth_frame
    
    def run(self):
        temp_frame = np.zeros((x_res, y_res))
        while self.running:
            # Wait for the next set of frames from the camera
            frames = self.pipe.wait_for_frames()
            depth = frames.get_depth_frame()
        
            for y in range(0, y_res):
                for x in range(100, x_res-100):
                    d = depth.get_distance(x, y)
                    if 0.3 < d < d_threshold:
                        temp_frame[x, y] = d
                    else:
                        temp_frame[x, y] = 0
            
            self.depth_frame = temp_frame
        
        self.pipe.stop()

"""    
class Calc(Thread):
    global y_fov, x_fov, x_res, y_res, d_threshold, slope_threshold 

    def __init__(self, frame_obj):
        self.frame_obj = frame_obj
        self.depth_frame = np.zeros((x_res, y_res, 3))
        self.running = True

        Thread.__init__(self)
    
    def get_calc_frame(self):
        return self.calc_frame
    
    def run(self):
        plane = np.zeros((x_res, y_res, 3))
        plane[:,:,0] = frame.get_depth_frame()

        for y in range(0, y_res-1):
            for x in range(0, x_res-1):
                d1 = plane[x, y, 0]
                d2 = plane[x, y+1, 0]
                d3 = plane[x+1, y, 0]

                if d1 != 0 and d2 != 0 and d3 != 0:
                    A = [d1*c_phi[x,y]*s_theta[x,y], d1*c_phi[x,y]*c_theta[x,y], d1*s_phi[x,y]]
                    B = [d2*c_phi[x,y+1]*s_theta[x,y+1], d2*c_phi[x,y+1]*c_theta[x,y+1], d2*s_phi[x,y+1]]
                    C = [d3*c_phi[x+1,y]*s_theta[x+1,y], d3*c_phi[x+1,y]*c_theta[x+1,y], d3*s_phi[x+1,y]]

                    N = np.cross([B[0]-A[0], B[1]-A[1], B[2]-A[2]], [C[0]-A[0], C[1]-A[1], C[2]-A[2]])
                    slope = m.acos(N[2]/np.linalg.norm(N))
                    if slope >= slope_threshold:
                        plane[x,y, 1] = slope
                        plane[x,y, 2] = 255
        
        self.calc_frame = plane
"""


def aruco_cb(data, args):
    global y_fov, x_fov, x_res, y_res, c_phi, s_phi, c_theta, s_theta, d_threshold, slope_threshold 
    
    z = args[1].get_depth_frame()
    d1 = z[data.x, data.y]
    pos = [d1*c_phi[data.x, data.y]*s_theta[data.x, data.y], d1*c_phi[data.x, data.y]*c_theta[data.x, data.y], d1*s_phi[data.x, data.y]]
    
    args[0].publish(int(pos[0]), int(pos[1]), d1, data.id)

# talker function
def talker():
    global x_fov, y_fov, x_res, y_res, c_phi, s_phi, c_theta, s_theta, d_threshold
    rospy.init_node('Object_Detection')
    x_fov = m.radians(rospy.get_param('~x_fov', 87))
    y_fov = m.radians(rospy.get_param('~y_fov', 58))
    x_res = rospy.get_param('~x_res', 480)
    y_res = rospy.get_param('~y_res', 270)
    d_threshold = rospy.get_param('~distance_threshold', 3)
    slope_threshold = rospy.get_param('~slope_threshold', 2)

    c_phi = np.zeros((x_res, y_res))
    s_phi = np.zeros((x_res, y_res))
    c_theta = np.zeros((x_res, y_res))
    s_theta = np.zeros((x_res, y_res))

    for y in range(0, y_res):
        for x in range(0, x_res):
            c_phi[x,y] = m.cos(y_fov*(y-y_res/2)/y_res)
            s_phi[x,y] = m.sin(y_fov*(y-y_res/2)/y_res)
            c_theta[x,y] = m.cos(x_fov*(x-x_res/2)/x_res)
            s_theta[x,y] = m.sin(x_fov*(x-x_res/2)/x_res)

    frame = Frames()
    frame.start()
    
    aruco_pub = rospy.Publisher("Aruco/depth", Aruco, queue_size=10)
    arcuo_sub = rospy.Subscriber('Aruco/pos', Aruco, aruco_cb, (aruco_pub, frame),queue_size=10)

    object_pub = rospy.Publisher("Objects", ObjectVector, queue_size=10)

    #calc = Calc()
    #calc.start()

    img = np.zeros((x_res, y_res))
    
    try:
        while not rospy.is_shutdown():
            plane = np.zeros((x_res, y_res, 3))
            plane[:,:,0] = frame.get_depth_frame()

            for y in range(y_res//2, y_res-1):
                for x in range(150, x_res-150-1):
                    d1 = plane[x, y, 0]
                    d2 = plane[x, y+1, 0]
                    #d3 = plane[x+1, y, 0]

                    if d1 != 0 and d2 != 0:
                        A = [d1*c_phi[x,y]*s_theta[x,y], d1*c_phi[x,y]*c_theta[x,y], d1*s_phi[x,y]]
                        B = [d2*c_phi[x,y+1]*s_theta[x,y+1], d2*c_phi[x,y+1]*c_theta[x,y+1], d2*s_phi[x,y+1]]
                        #C = [d3*c_phi[x+1,y]*s_theta[x+1,y], d3*c_phi[x+1,y]*c_theta[x+1,y], d3*s_phi[x+1,y]]

                        #N = np.cross([B[0]-A[0], B[1]-A[1], B[2]-A[2]], [C[0]-A[0], C[1]-A[1], C[2]-A[2]])
                        slope = m.acos((B[1]-A[1])/np.linalg.norm([B[0]-A[0], B[1]-A[1], B[2]-A[2]]))
                        #slope = m.acos(N[2]/np.linalg.norm(N))
                        if slope <= slope_threshold:
                            plane[x,y, 1] = slope
                            plane[x,y, 2] = 255
            
            img = np.array(plane[:,:,2].transpose(), dtype=np.uint8)
            thresh = cv2.adaptiveThreshold(img, 1, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)
            contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            #img = img.astype(np.uint8).copy()

            object_list = []
            object_data = []
            for cnt in contours:
                x,y,w,h = cv2.boundingRect(cnt)
                if h > 15:
                    overlap = False
                    for obj in object_list:
                        if obj[0] < x < obj[0] + obj[2]:
                            overlap = True
                            break
                    
                    if overlap is False:
                        object_list.append([x, y, w, h])
                        for x_inc in range(w//2):
                            d = plane[x+w//2+x_inc, y+h//2, 0]
                            if d != 0:
                                break
                        size = d*c_phi[x+w,y+h]*s_theta[x+w,y+h] - d*c_phi[x,y]*s_theta[x,y]
                        #cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), 2)
                        object_data.append(Object(d, x_fov*(x+w/2-x_res/2)/x_res, size))
            
            object_pub.publish(object_data)
            print(len(object_data))
            #cv2.imshow("Show", img)
            #cv2.waitKey(2000)
    
    finally:
        frame.running = False
        #calc.running = False
        frame.join()
        #calc.join()


# main loop
if __name__ == '__main__':
    
    talker()