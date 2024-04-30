from stitching import Stitcher
import cv2

dir_save = "/home/mavric/MAVRIC-Systems/src/mavric/src/PTZ_Control/PTZ_Photos/"

filename = dir_save + "pano.jpg"

stitcher = Stitcher(detector="sift", confidence_threshold=0.2)



pano = stitcher.stitch([dir_save + "Picture1.jpg", dir_save + "Picture2.jpg"])

cv2.imwrite(filename, pano)