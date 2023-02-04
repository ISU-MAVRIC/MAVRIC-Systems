#global constants
LIN_ERROR_THRESHOLD = 1     #arbitrary, meters
ANG_ERROR_THRESHOLD = 5    #arbitrary, degrees
ANG_POINT_STEER_MAX = 22    #between 1-100 (inclusive)
ANG_POINT_STEER_MIN = 5    #between 1-100 (inclusive)
LIN_DRIVE_MAX = 5          #between 1-100 (inclusive)
LIN_DRIVE_MIN = 1          #between 1-100 (inclusive)
LIN_DRIVE_STEER_MAX = 100    #between 1-100 (inclusive)
LIN_DRIVE_STEER_MIN = 35    #between 1-100 (inclusive)
ANG_POINT_STEER_TIMEOUT = 10000 #max ms until Turn state exit
UPDATE_INTERVAL = 1
FIX_TIMEOUT_THRESHOLD = UPDATE_INTERVAL * 5
PATH_BREAK_THRESHOLD = LIN_ERROR_THRESHOLD*3

#control flags
enabled = False
good_fix = False
good_imu = True
fix_timeout = False   #unnecessary?

#state
state = "Off"

#location data
position = [0, 0]
prev_position = [0, 0]
waypoints = []
waypoint_id = []
detected = False
angular_velocity = 0

# rover heading and position
heading = 0
desired_heading = 0

prev_angular_error = ANG_ERROR_THRESHOLD * 2
prev_linear_error = LIN_ERROR_THRESHOLD * 2

#pathing data
#PARAMS of pathpoint
# position: GPS postion of pathpoint
# linear: Drive towards waypoint or dynamic steer towards waypoint
# radius: turning radius for dynamic steering
# heading: desired rover heading when point is reached
# speed: speed when traveling towards waypoint
pathpoints  = {"position": [], "linear": [], "radius": [], "heading": [], "speed": []}

pathpoint_num = -1 # pathpoint rover has passed

path = None # object for pathing class

#post data
#PARAMS for post
# id: aruco tag number
# heading: angle from camera center
# distance: actual or approximate distance to post
# type: "realsense"  || "dome"
# pixel_location: Pixel location from camera
posts = {"id": [], "heading": [], "distance": [], "type": [], "pixel_location": []}

#object data
objects = {"distance": [], "heading": [], "size": [], "height": []}

#GPS timeout data
fix_time = 0
prev_fix_time = 0

#ROS params
Scale = 0
Rover_Width = 0
Rover_MinTurnRadius = 0

#ROS publishers (initialized in main)
drive_pub = None
steer_pub = None
debug_pub = None
status_pub = None
indicator_pub = None
aruco_pub = None