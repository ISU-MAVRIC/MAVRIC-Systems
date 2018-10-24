# First implementation of a base station script. This code runs the drive system
#       and the arm using a joystick and xbox controller, respectively.
#
#

#import modules
import pygame
import math
import socket
import phoenix

#declare constants
drive_stick_draw_pos = (250, 150)
shoulder_stick_draw_pos = (150, 350)
wrist_stick_draw_pos = (350, 350)
elbow_pitch_draw_pos = (50, 350)
claw_actuation_draw_pos = (450, 350)

temp_label_draw_pos = (25, 425)
location_label_draw_pos = (25, 450)
heading_label_draw_pos = (25, 475)
e_stop_label_draw_pos = (100, 15)
autonomous_label_draw_pos = (300, 450)

master_ip = '192.168.1.11'

drive_stick = None
drive_stick_name = "Mad Catz V.1 Stick" #hardcoded for now

arm_stick = None
arm_stick_name = "Logitech Dual Action" #hardcoded for now
#arm_stick_name = "Controller (XBOX 360 For Windows)" #hardcoded for now

#declare variables
joysticks = []

emergency_stop = False
autonomous_enabled = False

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

                elif event.key == pygame.K_e:
                    rover.enableAutonomous()
                    autonomous_enabled = True

                elif event.key == pygame.K_d:
                    rover.disableAutonomous()
                    autonomous_enabled = False

            #get shoulder and wrist commands from joysticks
            elif event.type == pygame.JOYAXISMOTION:
                #handle arm control axes
                if is_arm_stick(get_stick(event.joy)):
                    if event.axis == 0:
                        shoulder_r_axis = int(event.value * 100)
                        
                    elif event.axis == 1:
                        shoulder_p_axis = int(event.value * -100)
                        
                    elif event.axis == 2:
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

                    elif event.button == 1:
                        claw_a_val = -100

            elif event.type == pygame.JOYBUTTONUP:
                if is_arm_stick(get_stick(event.joy)):
                    if event.button == 3:
                        if get_stick(event.joy).get_button(1):
                            claw_a_val = -100
                        else:
                            claw_a_val = 0

                    elif event.button == 1:
                        if get_stick(event.joy).get_button(3):
                            claw_a_val = 100
                        else:
                            claw_a_val = 0

        #receive status data from rover
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
            
            rover.setArmBaseRot(shoulder_r_axis)
            rover.setArmBasePitch(shoulder_p_axis)
            rover.setArmElbowPitch(elbow_p_val)
            rover.setArmClawRot(wrist_r_axis)
            rover.setArmClawPitch(wrist_p_axis)
            rover.setArmClawActuation(claw_a_val)

        #draw UI
        if emergency_stop:
            e_stop_message = "Base station has been emergency stopped!"
            label = warn_font.render(e_stop_message, 1, (200, 0, 0))
            screen.blit(label, e_stop_label_draw_pos)

        if autonomous_enabled:
            autonomous_message = "Autonomous is active"
            label = warn_font.render(autonomous_message, 1, (200, 0, 200))
            screen.blit(label, autonomous_label_draw_pos)
        
        pygame.draw.rect(screen, (0,100,100), (drive_stick_draw_pos[0] - 100, drive_stick_draw_pos[1] - 100, 200, 200), 3)
        pygame.draw.rect(screen, (0,100,0), (shoulder_stick_draw_pos[0] - 50, shoulder_stick_draw_pos[1] - 50, 100, 100), 3)
        pygame.draw.rect(screen, (100,0,0), (wrist_stick_draw_pos[0] - 50, wrist_stick_draw_pos[1] - 50, 100, 100), 3)

        pygame.draw.line(screen, (100,100,0), (elbow_pitch_draw_pos[0], elbow_pitch_draw_pos[1] + 50), (elbow_pitch_draw_pos[0], elbow_pitch_draw_pos[1] - 50), 3)
        pygame.draw.line(screen, (100,0,100), (claw_actuation_draw_pos[0], claw_actuation_draw_pos[1] + 50), (claw_actuation_draw_pos[0], claw_actuation_draw_pos[1] - 50), 3)

        pygame.draw.circle(screen, (0,255,255), (drive_stick_draw_pos[0] + drive_x_axis, drive_stick_draw_pos[1] - drive_y_axis), 5)
        pygame.draw.circle(screen, (0,255,0), (shoulder_stick_draw_pos[0] + shoulder_r_axis/2, shoulder_stick_draw_pos[1] - shoulder_p_axis/2), 5)
        pygame.draw.circle(screen, (255,0,0), (wrist_stick_draw_pos[0] + wrist_r_axis/2, wrist_stick_draw_pos[1] - wrist_p_axis/2), 5)

        pygame.draw.circle(screen, (255,255,0), (elbow_pitch_draw_pos[0], elbow_pitch_draw_pos[1] - elbow_p_val/2), 5)
        pygame.draw.circle(screen, (255,0,255), (claw_actuation_draw_pos[0], claw_actuation_draw_pos[1] - claw_a_val/2), 5)

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
