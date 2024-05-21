#!/usr/bin/env python3

import rospy, time, cv2, os, glob, numpy as np
from imutils.video import VideoStream
from mavric.msg import Cam

commandTopic = "/Camera/Mast"
fbTopic = "/Camera/Mast_Feedback"
dir_save = "/home/nathan/MAVRIC-Systems/src/mavric/src/PTZ_Control/PTZ_Photos/"
positions = np.linspace(-0.5,0.5,10)
# positions = [-0.5, -0.25, 0, 0.25, 0.5]
ylevel = 0.518222
error = 0.02
X = 0
Y = 0
aspectRatio = 16/9

vs = VideoStream('rtsp://admin:mavric-camera@192.168.1.64:554/out.h264').start()

def pos_fb(data):
    global X,Y
    X = data.x
    Y = data.y

rospy.init_node("Panoramic")
command = rospy.Publisher(commandTopic, Cam, queue_size=10)
fb = rospy.Subscriber(fbTopic, Cam, pos_fb)
rate = rospy.Rate(1)
time.sleep(1)

def capture():
    global positions
    print("Capturing Photos")
    os.chdir(dir_save)
    for file in glob.glob("*.jpg"):
        os.remove(file)
    n = 0
    positions = np.around(positions, 4)
    for pos in positions:
        request = Cam(pos,ylevel)
        command.publish(request)
        while abs(pos-X) > error and not rospy.is_shutdown():
            time.sleep(0.25)
        time.sleep(2)
        frame = vs.read()
        cv2.imwrite(dir_save + str(n) + ".jpg", frame)
        n = n + 1
    print("Finished Captures")

def stitcher():
    print("Stitching Photos")
    os.chdir(dir_save)
    imgs = []
    for file in glob.glob("*.jpg"):
        imgs.append(cv2.imread(file))
    stitch = cv2.Stitcher.create()
    (dummy,output) = stitch.stitch(imgs)
    height, width, layers = output.shape
    output = cv2.resize(output, (round(width/6),height),interpolation=cv2.INTER_AREA)
    cv2.imwrite(dir_save + "FINAL.jpg", output)
    print("Finished Stitching")

if __name__ == '__main__':
    try:
        capture()
        vs.stop()
        stitcher()

    except rospy.ROSInterruptException:
        vs.stop()
        print("Quitting")
        pass