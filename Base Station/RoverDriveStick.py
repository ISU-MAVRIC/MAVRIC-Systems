# This code is meant to run the wheels on the Pheonix rover and nothing more.
# Isaiah Exley-Schuman
# 7-12-2018

# initialize and import
import pygame # for controller
import math
import socket   # for sockets
import sys  # for exit
import phoenix

pygame.init()
pygame.joystick.init()
joysticks = []
rover = phoenix.Phoenix('192.168.1.11')

#try:
#        # create TCP link socket
#        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#except socket.error:
#        print ('Failed to create socket.')
#        sys.exit();
#
#print ('Socket Created')

ip = "192.168.1.11"
port = 9002

# connect to rover
#s.connect((ip , port))
 
#print ('Socket Connected to ip ' + ip)

# open and name viewing window
screen = pygame.display.set_mode((500,500))
pygame.display.set_caption("Rover Driving Code")

# find controller
joysticks = []
for i in range(0, pygame.joystick.get_count()):
        # create a Joystick object in our list
        joysticks.append(pygame.joystick.Joystick(i))
        # initialize them all (-1 means loop forever)
        joysticks[-1].init()
        # print a statement telling what the name of the controller is
        print ("Detected joystick '",joysticks[-1].get_name(),"'")

# run main loop
run = True
R = 0
L = 0
axis1 = 0
axis0 = 0
rover.open()
actuator_enabled = False
try:
    
    # initialize font; must be called after 'pygame.init()' to avoid 'Font not Initialized' error
    myfont = pygame.font.SysFont("Arial", 15)
    while run:
        # loop every .1 seconds
        pygame.time.delay(100)
        rover.setWheels(L,R)        
        screen.fill((0,0,0))
        
        try:
            temp_message = "Temperature: %0.2f C" % (rover.temperature)
            label = myfont.render(temp_message, 1, (255,255,0))
            screen.blit(label, (100, 100))
        except:
            pass
        
        try:
            temp_message = "Actuator: ~%0.2f" % (rover.actuator)
            label = myfont.render(temp_message, 1, (255,255,0))
            screen.blit(label, (100, 200))
        except:
            pass
        pygame.display.flip()
        
        for event in pygame.event.get():
            # end program if red x pressed
            if event.type == pygame.QUIT:
                run = False
            # detect button motion
            elif event.type == pygame.JOYAXISMOTION:
                #print(event)
                if event.axis == 1:
                    axis1 = event.value;
                if event.axis == 0:
                    axis0 = event.value;
                    
                val2 = int(-axis0*50)
                val1 = int(-axis1*100)
                L = val1-val2
                R = val1+val2

            elif event.type == pygame.JOYHATMOTION:
                rover.setActuator(event.value[1])
                
    pygame.quit()
except Exception as e:
    print(e)
    pass
finally:
    rover.close()