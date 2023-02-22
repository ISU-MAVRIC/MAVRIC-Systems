#global constants
LIN_ERROR_THRESHOLD = 1     # How close the rover has to be to a waypoint to have reached it, meters
ANG_ERROR_THRESHOLD = 5    # Allowed deviation from the target heading, degrees
#ADVISOR NOTE: The max velocity constants are percent outputs, and need to be changed to work with velocity control
ANG_POINT_STEER_MAX = 22    # Maximum velocity when point steering, between 1-100 percent output
ANG_POINT_STEER_MIN = 5    # Minumum velocity when point steering between 1-100 percent output
LIN_DRIVE_MAX = 5          # Maximum velocity when tank driving, between 1-100 percent output
LIN_DRIVE_MIN = 1          # Minumum velocity when tank driving between 1-100 percent output
LIN_DRIVE_STEER_MAX = 100    # Maximum velocity when dynamic driving, between 1-100 percent output
LIN_DRIVE_STEER_MIN = 35    # Minumum velocity when dynamic driving between 1-100 percent output
ANG_POINT_STEER_TIMEOUT = 10000 # max time in ms until turn towards pathpoint state exit
UPDATE_INTERVAL = 1
FIX_TIMEOUT_THRESHOLD = UPDATE_INTERVAL * 5
PATH_BREAK_THRESHOLD = LIN_ERROR_THRESHOLD*3

#control flags
enabled = False
good_fix = False
good_imu = True
fix_timeout = False   #unnecessary?

#location data
position = [0, 0]
prev_position = [0, 0]
waypoints = []
waypoint_id = []
detected = False

heading = 0

prev_angular_error = ANG_ERROR_THRESHOLD * 2
prev_linear_error = LIN_ERROR_THRESHOLD * 2

#pathing data
#PARAMS of pathpoint
# position: GPS postion of pathpoint
# linear: Drive towards waypoint or bank steer towards waypoint
# radius: turning radius for bank steering
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