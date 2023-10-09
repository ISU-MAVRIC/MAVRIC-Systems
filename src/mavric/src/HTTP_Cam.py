import subprocess
import time
import cv2


def detect_camera_location():
    camera_location = 0 # Default camera location is 0
    for i in range(1):  # Checks for camera on video0 or video1
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            camera_location = i
            cap.release()
            break
    return camera_location

def shutdown_mjpg_streamer():
    try:
        subprocess.run(["pkill", "-f", "mjpg_streamer"])
        print("MJPG-Streamer has been shut down.")
    except Exception as e:
        print("Error shutting down MJPG-Streamer")


camera_location = detect_camera_location()

# Camera input settings
resolution = '1920x1080'
frame_rate = '60'

# Creates HTTP Stream from location of the camera using MJPG Streamer
mjpg_streamer_command = f"mjpg_streamer -i 'input_uvc.so -d /dev/video{camera_location} -r {resolution} -f {frame_rate}' -o 'output_http.so -w /home/mavric/mjpg-streamer/mjpg-streamer-experimental/www'"

# Starts MJPG Streamer as a subprocess
try:
    subprocess.Popen(mjpg_streamer_command, shell=True)
    print("MJPG Streamer is running")
except:
    print("There was an error trying to run MJPG Streamer.")


# Checks for keyboard inturpt every second and then will kill MJPG Streamer process
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    shutdown_mjpg_streamer()