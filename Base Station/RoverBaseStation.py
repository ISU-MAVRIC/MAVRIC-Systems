# First implementation of a base station script. This code runs the drive system
#       and the arm using a joystick and xbox controller, respectively.
#
#

#import modules
import pygame
import math
import socket
import phoenix

from geographiclib.geodesic import Geodesic

#graphics constants
drive_stick_draw_pos = (425, 75)
shoulder_stick_draw_pos = (125, 100)
wrist_stick_draw_pos = (225, 100)
elbow_pitch_draw_pos = (50, 100)
claw_actuation_draw_pos = (300, 100)

drive_stick_draw_size = 100
shoulder_stick_draw_size = 50
wrist_stick_draw_size = 50
elbow_pitch_draw_size = 50
claw_actuation_draw_size = 50

temp_label_draw_pos = (25, 425)
location_label_draw_pos = (25, 450)
heading_label_draw_pos = (25, 475)
e_stop_label_draw_pos = (100, 15)
autonomous_label_draw_pos = (300, 450)

autonomous_map_N_label_draw_pos = (325, 155)
autonomous_map_draw_pos = (325, 325)
autonomous_map_draw_size = 300

rover_icon_radius = 5
rover_icon_fin_angle = 135

#system constants
master_ip = '192.168.1.10'

drive_stick = None
drive_stick_name = "Mad Catz V.1 Stick" #hardcoded for now

arm_stick = None
#arm_stick_name = "Logitech Dual Action" #hardcoded for now
arm_stick_name = "Controller (XBOX 360 For Windows)" #hardcoded for now

autonomous_map_scale = 100  #width of the map in meters

#declare variables
joysticks = []

emergency_stop = False
autonomous_enabled = False

autonomous_map_center_pos = []  #(lat, long) obtained from the rover at startup
#autonomous_map_center_pos = [42.034534, -93.620369]
waypoints = []
waypoints_draw = []

autonomous_rover_pos = []  #(lat, long) obtained from the rover periodically
#autonomous_rover_pos = autonomous_map_center_pos
autonomous_rover_heading = 0

drive_x_axis = 0
drive_y_axis = 0
shoulder_r_axis = 0
shoulder_p_axis = 0
wrist_r_axis = 0
wrist_p_axis = 0
elbow_p_val = 0
claw_a_val = 0

#declare helper functions
def get_stick(stick_id):
    return pygame.joystick.Joystick(stick_id)

def is_drive_stick(stick):
    #catch uninitialized sticks
    if stick is None:
        return False

    if drive_stick is None:
        return False

    #compare against known drive stick
    if stick.get_id() == drive_stick.get_id():
        return True

    return False

def is_arm_stick(stick):
    #catch uninitialized sticks
    if stick is None:
        return False

    if arm_stick is None:
        return False

    #compare against known arm stick
    if stick.get_id() == arm_stick.get_id():
        return True

    return False

def add_autonomous_waypoint(mouse_click_pos):
    #ignore clicks outside the autonomous map frame
    if mouse_click_pos[0] < autonomous_map_draw_pos[0] - autonomous_map_draw_size/2 \
       or mouse_click_pos[0] > autonomous_map_draw_pos[0] + autonomous_map_draw_size/2:
        return

    if mouse_click_pos[1] < autonomous_map_draw_pos[1] - autonomous_map_draw_size/2 \
       or mouse_click_pos[1] > autonomous_map_draw_pos[1] + autonomous_map_draw_size/2:
        return

    #ignore clicks if GPS data from the rover is not available yet
    if autonomous_map_center_pos == []:
        return

    #convert to map-relative screen coordinates
    #   assumes the center of the map is (0, 0)
    map_click_pos = [0, 0]
    map_click_pos[0] = mouse_click_pos[0] - autonomous_map_draw_pos[0]
    map_click_pos[1] = mouse_click_pos[1] - autonomous_map_draw_pos[1]

    #convert to meters
    map_pos_meters = map_click_pos
    map_pos_meters[0] = int((float(map_click_pos[0]) / autonomous_map_draw_size) * autonomous_map_scale)
    map_pos_meters[1] = int((float(map_click_pos[1]) / autonomous_map_draw_size) * autonomous_map_scale)

    #calculate distance and angle of waypoint from rover
    s12 = math.sqrt(map_pos_meters[0]**2 + map_pos_meters[1]**2)
    azi = math.degrees(math.atan2(map_pos_meters[1], map_pos_meters[0]))

    #convert screen angle to azimuth (clockwise degrees from north)
    azi += 90 + 360
    azi %= 360

    #solve the direct geodesic problem to find the second point
    #   assumes WGS84 ellipsoid model
    geod = Geodesic.WGS84.Direct(autonomous_map_center_pos[0],
                                 autonomous_map_center_pos[1], azi, s12)

    map_pos_gps = [0, 0]
    map_pos_gps[0] = round(geod['lat2'], 6) #gps only reports to 6 decimals,
    map_pos_gps[1] = round(geod['lon2'], 6) #   maintain consistency

    #store waypoint and send to rover
    waypoints.append(map_pos_gps)
    waypoints_draw.append(gps_to_screen_pos(map_pos_gps))

    rover.addWaypoint(float(map_pos_gps[0]), float(map_pos_gps[1]))

def gps_to_screen_pos(gps_pos):
    #solve the inverse geodesic problem to find the rover's position in meters
    geod = Geodesic.WGS84.Inverse(autonomous_map_center_pos[0],
                                  autonomous_map_center_pos[1],
                                  gps_pos[0], gps_pos[1])

    s12 = geod['s12']
    azi = geod['azi1']

    #calculate screen pos in meters
    rover_pos_meters = [0, 0]
    rover_pos_meters[0] = s12 * math.sin(math.radians(azi))
    rover_pos_meters[1] = s12 * -math.cos(math.radians(azi))

    #convert to map-relative screen coordinates
    rover_map_pos = [0, 0]
    rover_map_pos[0] = int((rover_pos_meters[0] / autonomous_map_scale) * autonomous_map_draw_size)
    rover_map_pos[1] = int((rover_pos_meters[1] / autonomous_map_scale) * autonomous_map_draw_size)

    #convert to screen pos
    rover_screen_pos = [0, 0]
    rover_screen_pos[0] = rover_map_pos[0] + autonomous_map_draw_pos[0]
    rover_screen_pos[1] = rover_map_pos[1] + autonomous_map_draw_pos[1]

    return rover_screen_pos

def draw_rover_icon(screen_pos, heading):
    fd_pt = [0, 0]
    bk_lf_pt = [0, 0]
    bk_rt_pt = [0, 0]

    #skip drawing if the rover has not acquired a position yet
    if screen_pos == []:
        return

    #convert azimuth to screen angle
    heading -= 90

    #calculate forward point
    fd_pt[0] = screen_pos[0] + int(rover_icon_radius * math.cos(math.radians(heading)))
    fd_pt[1] = screen_pos[1] + int(rover_icon_radius * math.sin(math.radians(heading)))

    #calculate fin angles
    bk_lf_h = heading + rover_icon_fin_angle
    bk_rt_h = heading - rover_icon_fin_angle

    #calculate fin points
    bk_lf_pt[0] = screen_pos[0] + int(rover_icon_radius * math.cos(math.radians(bk_lf_h)))
    bk_lf_pt[1] = screen_pos[1] + int(rover_icon_radius * math.sin(math.radians(bk_lf_h)))

    bk_rt_pt[0] = screen_pos[0] + int(rover_icon_radius * math.cos(math.radians(bk_rt_h)))
    bk_rt_pt[1] = screen_pos[1] + int(rover_icon_radius * math.sin(math.radians(bk_rt_h)))

    #draw icon
    pygame.draw.polygon(screen, (255,0,0), [screen_pos, bk_rt_pt, fd_pt, bk_lf_pt], 0)

#initialize pygame
pygame.init()
pygame.joystick.init()

screen = pygame.display.set_mode((500, 500))
pygame.display.set_caption("MAVRIC Base Station")

info_font = pygame.font.SysFont("Arial", 15)
warn_font = pygame.font.SysFont("Arial", 20)

#find controllers
for i in range(0, pygame.joystick.get_count()):
    joysticks.append(pygame.joystick.Joystick(i))
    joysticks[-1].init()

    #select drive stick based on name
    #   this means using a different stick will require the hardcoded name to change
    if joysticks[-1].get_name() == drive_stick_name:
        print("Detected drive joystick '%s'" % joysticks[-1].get_name())
        drive_stick = joysticks[-1]

    #select drive controller based on name
    #   this means using a different controller will require the hardcoded name to change
    elif joysticks[-1].get_name() == arm_stick_name:
        print("Detected arm controller '%s'" % joysticks[-1].get_name())
        arm_stick = joysticks[-1]

    else:
        print("Detected unbound controller '%s'" % joysticks[-1].get_name())

#start main loop
rover = phoenix.Phoenix(master_ip)
rover.open()
rover.killAll()

run = True
try:
    while run:
        #process game events
        for event in pygame.event.get():
            #if red 'X' is clicked, quit program
            if event.type == pygame.QUIT:
                run = False

            #handle keyboard events
            if event.type == pygame.KEYDOWN:
                #software emergency stop
                if event.key == pygame.K_SPACE:
                    rover.killAll()
                    emergency_stop = True
                    autonomous_enabled = False

                #software emergency stop reset
                elif event.key == pygame.K_RETURN:
                    emergency_stop = False

                elif event.key == pygame.K_w:
                    pt_str = raw_input("Waypoint (lat, lon): ")
                    pt = pt_str.strip("() ").split(',')
                    rover.addWaypoint(float(pt[0]), float(pt[1]))

                elif event.key == pygame.K_c:
                    #re-center autonomous map
                    autonomous_map_center_pos = []

                elif event.key == pygame.K_e:
                    rover.enableAutonomous()
                    autonomous_enabled = True

                elif event.key == pygame.K_d:
                    rover.disableAutonomous()
                    autonomous_enabled = False

            #handle mouse events
            elif event.type == pygame.MOUSEBUTTONDOWN:
                #left click
                if pygame.mouse.get_pressed()[0]:
                    add_autonomous_waypoint(pygame.mouse.get_pos())

            #get shoulder and wrist commands from joysticks
            elif event.type == pygame.JOYAXISMOTION:
                #handle arm control axes
                if is_arm_stick(get_stick(event.joy)):
                    if event.axis == 0:
                        shoulder_r_axis = int(event.value * 100)
                        
                    elif event.axis == 1:
                        shoulder_p_axis = int(event.value * -100)
                        
                    elif event.axis == 4:
                        wrist_r_axis = int(event.value * 100)

                    elif event.axis == 3:
                        wrist_p_axis = int(event.value * -100)

                #handle drive control axes
                if is_drive_stick(get_stick(event.joy)):
                    #if event.axis == 0:    #tilt
                    if event.axis == 3:     #twist
                        drive_x_axis = int(event.value * 100)

                    elif event.axis == 1:
                        drive_y_axis = int(event.value * -100)

            #get elbow command from arm controller d-pad
            elif event.type == pygame.JOYHATMOTION:
                if is_arm_stick(get_stick(event.joy)):
                    elbow_p_val = event.value[1] * 100

            #get claw grip commands from arm controller button pad
            elif event.type == pygame.JOYBUTTONDOWN:
                if is_arm_stick(get_stick(event.joy)):
                    if event.button == 3:
                        claw_a_val = 100

                    elif event.button == 0:
                        claw_a_val = -100

            elif event.type == pygame.JOYBUTTONUP:
                if is_arm_stick(get_stick(event.joy)):
                    if event.button == 3:
                        if get_stick(event.joy).get_button(1):
                            claw_a_val = -100
                        else:
                            claw_a_val = 0

                    elif event.button == 0:
                        if get_stick(event.joy).get_button(3):
                            claw_a_val = 100
                        else:
                            claw_a_val = 0

        #receive status data from rover
        try:
            if autonomous_map_center_pos == []:
                autonomous_map_center_pos = (rover.gps.latitude, rover.gps.longitude)

            autonomous_rover_pos = (rover.gps.latitude, rover.gps.longitude)
            autonomous_rover_heading = rover.gps.heading
        except:
            pass

        #display status data from rover
        try:
            temp_message = "Temperature: %0.2f C" % (rover.temperature)
            label = info_font.render(temp_message, 1, (255,255,0))
            screen.blit(label, temp_label_draw_pos)

            location_message = "Position: (%0.6f, %0.6f)" % (rover.gps.latitude, rover.gps.longitude)
            label = info_font.render(location_message, 1, (255,255,0))
            screen.blit(label, location_label_draw_pos)

            heading_message = "Heading: %0.2f CW" % (rover.gps.heading)
            label = info_font.render(heading_message, 1, (255,255,0))
            screen.blit(label, heading_label_draw_pos)
        except:
            pass

        #convert from joystick arcade drive to rover tank drive
        #   add condition to make backing up behave the way you'd expect (like a car)
        drive_l = drive_y_axis + (drive_x_axis/2)
        drive_r = drive_y_axis - (drive_x_axis/2)

        """if drive_y_axis >= 0:
            drive_l = drive_y_axis + (drive_x_axis/2)
            drive_r = drive_y_axis - (drive_x_axis/2)

        else:
            drive_l = drive_y_axis - (drive_x_axis/2)
            drive_r = drive_y_axis + (drive_x_axis/2)"""

        #send commands to rover if safe to do so
        if not emergency_stop:
            rover.setWheels(drive_l, drive_r)
            
            #rover.setArmBaseRot(shoulder_r_axis)
            #rover.setArmBasePitch(shoulder_p_axis)
            #rover.setArmElbowPitch(elbow_p_val)
            #rover.setArmClawRot(wrist_r_axis)
            #rover.setArmClawPitch(wrist_p_axis)
            #rover.setArmClawActuation(claw_a_val)

        #draw UI
        if emergency_stop:
            e_stop_message = "Base station has been emergency stopped!"
            label = warn_font.render(e_stop_message, 1, (200, 0, 0))
            screen.blit(label, e_stop_label_draw_pos)

        if autonomous_enabled:
            autonomous_message = "Autonomous is active"
            label = warn_font.render(autonomous_message, 1, (200, 0, 200))
            screen.blit(label, autonomous_label_draw_pos)

        #draw axis frames
        pygame.draw.rect(screen, (0,100,100), (drive_stick_draw_pos[0] - int(drive_stick_draw_size/2),
                                               drive_stick_draw_pos[1] - int(drive_stick_draw_size/2),
                                               drive_stick_draw_size, drive_stick_draw_size), 3)
        
        pygame.draw.rect(screen, (0,100,0), (shoulder_stick_draw_pos[0] - int(shoulder_stick_draw_size/2),
                                             shoulder_stick_draw_pos[1] - int(shoulder_stick_draw_size/2),
                                             shoulder_stick_draw_size, shoulder_stick_draw_size), 3)
        
        pygame.draw.rect(screen, (100,0,0), (wrist_stick_draw_pos[0] - int(wrist_stick_draw_size/2),
                                             wrist_stick_draw_pos[1] - int(wrist_stick_draw_size/2),
                                             wrist_stick_draw_size, wrist_stick_draw_size), 3)

        #draw axis lines
        pygame.draw.line(screen, (100,100,0), (elbow_pitch_draw_pos[0], int(elbow_pitch_draw_pos[1] + elbow_pitch_draw_size/2)),
                                              (elbow_pitch_draw_pos[0], int(elbow_pitch_draw_pos[1] - elbow_pitch_draw_size/2)), 3)
        
        pygame.draw.line(screen, (100,0,100), (claw_actuation_draw_pos[0], int(claw_actuation_draw_pos[1] + claw_actuation_draw_size/2)),
                                              (claw_actuation_draw_pos[0], int(claw_actuation_draw_pos[1] - claw_actuation_draw_size/2)), 3)

        #draw controller points
        pygame.draw.circle(screen, (0,255,255), (drive_stick_draw_pos[0] + int(drive_x_axis * ((drive_stick_draw_size/2) / 100.0)),
                                                 drive_stick_draw_pos[1] - int(drive_y_axis * ((drive_stick_draw_size/2) / 100.0))), 5)
        
        pygame.draw.circle(screen, (0,255,0), (shoulder_stick_draw_pos[0] + int(shoulder_r_axis * ((shoulder_stick_draw_size/2) / 100.0)),
                                               shoulder_stick_draw_pos[1] - int(shoulder_p_axis * ((shoulder_stick_draw_size/2) / 100.0))), 5)
        
        pygame.draw.circle(screen, (255,0,0), (wrist_stick_draw_pos[0] + int(wrist_r_axis * ((wrist_stick_draw_size/2) / 100.0)),
                                               wrist_stick_draw_pos[1] - int(wrist_p_axis * ((wrist_stick_draw_size/2) / 100.0))), 5)

        pygame.draw.circle(screen, (255,255,0), (elbow_pitch_draw_pos[0],
                                                 elbow_pitch_draw_pos[1] - int(elbow_p_val * ((elbow_pitch_draw_size/2) / 100.0))), 5)
        
        pygame.draw.circle(screen, (255,0,255), (claw_actuation_draw_pos[0],
                                                 claw_actuation_draw_pos[1] - int(claw_a_val * ((claw_actuation_draw_size/2) / 100.))), 5)

        #draw autonomous frame
        pygame.draw.rect(screen, (100,100,0), (autonomous_map_draw_pos[0] - int(autonomous_map_draw_size/2),
                                               autonomous_map_draw_pos[1] - int(autonomous_map_draw_size/2),
                                               autonomous_map_draw_size, autonomous_map_draw_size), 3)

        #draw north label
        north_label = info_font.render("N", 1, (255,255,0))
        screen.blit(north_label, autonomous_map_N_label_draw_pos)

        #draw autonomous start point
        pygame.draw.circle(screen, (255,255,0), autonomous_map_draw_pos, 2)

        #draw autonomous waypoints and lines between them
        for w in range(0, len(waypoints_draw)):
            if w == 0:
                pygame.draw.line(screen, (100,100,0), autonomous_map_draw_pos, waypoints_draw[w], 1)

            elif w > 0:
                pygame.draw.line(screen, (100,100,0), waypoints_draw[w-1], waypoints_draw[w], 1)

            pygame.draw.circle(screen, (255,255,0), waypoints_draw[w], 5)

        #draw rover on autonomous map
        if autonomous_rover_pos != []:
            draw_rover_icon(gps_to_screen_pos(autonomous_rover_pos), autonomous_rover_heading)
        
        pygame.display.flip()
        
        #loop every 0.05 seconds (20 Hz)
        pygame.time.delay(50)
        screen.fill((0,0,0))

except Exception as e:
    print(e)

finally:
    pygame.quit()
    rover.killAll()
    rover.close()
