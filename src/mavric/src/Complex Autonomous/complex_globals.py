#global constants
LIN_ERROR_THRESHOLD = 5     #arbitrary, meters
ANG_ERROR_THRESHOLD = 10    #arbitrary, degrees
ANG_POINT_STEER_MAX = 70    #between 1-100 (inclusive)
ANG_POINT_STEER_MIN = 10    #between 1-100 (inclusive)
LIN_DRIVE_MAX = 90          #between 1-100 (inclusive)
LIN_DRIVE_MIN = 20          #between 1-100 (inclusive)
LIN_DRIVE_STEER_MAX = 70    #between 1-100 (inclusive)
LIN_DRIVE_STEER_MIN = 20    #between 1-100 (inclusive)
ANG_POINT_STEER_TIMEOUT = 10000 #max ms until Turn state exit
UPDATE_INTERVAL = 1
FIX_TIMEOUT_THRESHOLD = UPDATE_INTERVAL * 5
PATH_BREAK_THRESHOLD = LIN_ERROR_THRESHOLD*3

#control flags
enabled = False
good_fix = False
good_imu = False
fix_timeout = False   #unnecessary?

#location data
position = [0, 0]
prev_position = [0, 0]
waypoints = []
waypoint_id = []

heading = 0

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

pathpoint_num = 0 # pathpoint rover has passed

#post data
#PARAMS for post
# id: aruco tag number
# heading: angle from camera center
# distance: actual or approximate distance to post
posts = {"id": [], "heading": [], "distance": []}

#object data
objects = {"position": [], "size": [], "height": []}

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
indicator_pub = None