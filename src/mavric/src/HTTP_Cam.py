#!/usr/bin/env python3

import subprocess
import rospy
import time

# Shuts down the MJPG Subprocess
def shutdown_mjpg_streamer():
    try:
        subprocess.run(["pkill", "-f", "mjpg_streamer"])
        print("MJPG-Streamer has been shut down.")
    except Exception as e:
        print("Error shutting down MJPG-Streamer")
        
if __name__ == '__main__':
    try:
        # Camera input settings from ROS Camera node
        rospy.init_node('arm_camera')
        resolution = rospy.get_param('~resolution', '1280x720')
        frame_rate = rospy.get_param('~frame_rate', '30')
        port = rospy.get_param('~port', '8080')
        camera_location = rospy.get_param('~video_device', '0')

        # Command for creating HTTP Stream of a camera using MJPG Streamer based on ROS parameters
        mjpg_streamer_command = f"mjpg_streamer -i 'input_uvc.so -d /dev/video{camera_location} -r {resolution} -f {frame_rate}' -o 'output_http.so -w /home/mavric/mjpg-streamer/mjpg-streamer-experimental/www -p {port}'"

        # Runs the MJPG Stream command as a subprocess
        try:
            subprocess.Popen(mjpg_streamer_command, shell=True)
            print("MJPG Streamer is running")
        except:
            print("There was an error trying to run MJPG Streamer.")

        # Keeps MJPG running if there's no exception
        while True:
            time.sleep(1)

    # Calls the MJPG Shutdown function if there is an interruption from ROS
    except rospy.ROSInterruptException:
        shutdown_mjpg_streamer()
        pass
