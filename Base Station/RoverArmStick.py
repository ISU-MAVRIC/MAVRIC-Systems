# This code runs the arm on the Phoenix rover.

#initialize and import
import pygame  #for controller
import math
import socket  #for sockets
import sys     #for exit
import phoenix

pygame.init()
pygame.joystick.init()
joysticks = []
rover = phoenix.Phoenix('192.168.1.11')

ip = "192.168.1.11"
port = 9002

drive_stick_name = ""

shoulder_stick_draw_pos = (150, 350)
wrist_stick_draw_pos = (350, 350)
elbow_pitch_draw_pos = (50, 350)
claw_actuation_draw_pos = (450, 350)

temp_label_draw_pos = (100, 100)

#open window
screen = pygame.display.set_mode((500, 500))
pygame.display.set_caption("Rover Arm Driving Code")

#find xbox controller
for i in range(0, pygame.joystick.get_count()):
    joysticks.append(pygame.joystick.Joystick(i))
    joysticks[-1].init()

    print("Detected joystick '", joysticks[-1].get_name(), "'")

#start main loop
run = True

shoulder_r_axis = 0
shoulder_p_axis = 0
wrist_r_axis = 0
wrist_p_axis = 0
elbow_p_val = 0
claw_a_val = 0

rover.open()

try:
    #initialize UI font
    info_font = pygame.font.SysFont("Arial", 15)
    
    while run:
        #process game events
        for event in pygame.event.get():
            #if window 'X' is clicked, quit program
            if event.type == pygame.QUIT:
                run = False

            #get shoulder and wrist commands from joysticks
            elif event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:
                    shoulder_r_axis = int(event.value * 100)
                    
                elif event.axis == 1:
                    shoulder_p_axis = int(event.value * -100)
                    
                elif event.axis == 2:
                    wrist_r_axis = int(event.value * 100)

                elif event.axis == 3:
                    wrist_p_axis = int(event.value * -100)

            #get elbow command from d-pad
            elif event.type == pygame.JOYHATMOTION:
                elbow_p_val = event.value[1] * 100

            #get claw grip commands from button pad
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 3:
                    claw_a_val = 100

                elif event.button == 1:
                    claw_a_val = -100

            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 3:
                    if pygame.joystick.Joystick(event.joy).get_button(1):
                        claw_a_val = -100
                    else:
                        claw_a_val = 0

                elif event.button == 1:
                    if pygame.joystick.Joystick(event.joy).get_button(3):
                        claw_a_val = 100
                    else:
                        claw_a_val = 0

        #receive data from rover
        try:
            temp_message = "Temperature: %0.2f C" % (rover.temperature)
            label = info_font.render(temp_message, 1, (255,255,0))
            screen.blit(label, temp_label_draw_pos)
        except:
            pass

        #print(shoulder_r_axis, shoulder_p_axis, elbow_p_val, wrist_r_axis, wrist_p_axis, claw_a_val)

        #send commands to rover
        rover.setArmBaseRot(shoulder_r_axis)
        rover.setArmBasePitch(shoulder_p_axis)
        rover.setArmElbowPitch(elbow_p_val)
        rover.setArmClawRot(wrist_r_axis)
        rover.setArmClawPitch(wrist_p_axis)
        rover.setArmClawActuation(claw_a_val)

        #draw UI
        pygame.draw.rect(screen, (0,100,0), (shoulder_stick_draw_pos[0] - 50, shoulder_stick_draw_pos[1] - 50, 100, 100), 3)
        pygame.draw.rect(screen, (100,0,0), (wrist_stick_draw_pos[0] - 50, wrist_stick_draw_pos[1] - 50, 100, 100), 3)

        pygame.draw.line(screen, (100,100,0), (elbow_pitch_draw_pos[0], elbow_pitch_draw_pos[1] + 50), (elbow_pitch_draw_pos[0], elbow_pitch_draw_pos[1] - 50), 3)
        pygame.draw.line(screen, (100,0,100), (claw_actuation_draw_pos[0], claw_actuation_draw_pos[1] + 50), (claw_actuation_draw_pos[0], claw_actuation_draw_pos[1] - 50), 3)

        pygame.draw.circle(screen, (0,255,0), (shoulder_stick_draw_pos[0] + shoulder_r_axis/2, shoulder_stick_draw_pos[1] - shoulder_p_axis/2), 5)
        pygame.draw.circle(screen, (255,0,0), (wrist_stick_draw_pos[0] + wrist_r_axis/2, wrist_stick_draw_pos[1] - wrist_p_axis/2), 5)

        pygame.draw.circle(screen, (255,255,0), (elbow_pitch_draw_pos[0], elbow_pitch_draw_pos[1] - elbow_p_val/2), 5)
        pygame.draw.circle(screen, (255,0,255), (claw_actuation_draw_pos[0], claw_actuation_draw_pos[1] - claw_a_val/2), 5)

        pygame.display.flip()

        #loop every 0.05 seconds (20 Hz)
        pygame.time.delay(50)
        screen.fill((0,0,0))

    pygame.quit()

except Exception as e:
    print(e)

finally:
    rover.close()
    pass
