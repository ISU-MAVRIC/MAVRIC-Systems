# install modules
import pygame as py
import pygame.display as display
import pygame.draw as draw
import pygame.joystick as joystick
import math as m
import numpy as num
import matplotlib.pyplot as plot

# initial values
Lower_L = 1
Upper_L = 1
Claw_L = 0.1
Lower_C = [0, 255, 255]
Upper_C = [0, 255, 255]
Claw_C = [0, 255, 255]

theta = 0
phi = 0
gamma = 0
omega = 0
alpha = 0
mu = 0
a = 0
b = 0
c = 0
B = 0

Shoulder_Pos = num.array([0.0, 0.0])
Elbow_Pos = num.zeros(2)
Wrist_Pos = num.zeros(2)
Claw_Pos = num.array([1.0, 1.0])
offset = num.array([250, 100])
Shoulder_Coord = num.zeros(2)
Elbow_Coord = num.zeros(2)
Wrist_Coord = num.zeros(2)

# pygame values
controller = 0
use_cont = False
py.init()
joystick.init()
clock = py.time.Clock()
win = display.set_mode((1500, 800), py.RESIZABLE)
display.set_caption('MAVRIC Arm Simulator')
infofont = py.font.SysFont("Arial", 15)

# control values
m_to_pixels = 500
x_move = False
a_move = False
z_move = False
x_scale = 1
a_scale = 1
z_scale = 1

# data values
record = False
file_name = "Data"
file_number = 0
file = None
delta = 0
t = 0
start_time = 0

# get joystick
if joystick.get_count() == 1:
    controller = joystick.Joystick(1)
    use_cont = True
else:
    print("No Controller")

run = True
while run:
    # process game events
    for event in py.event.get():
        #print(event)
        # if red 'X' is clicked, quit program
        if event.type == py.QUIT:
            run = False
        # if window is resized
        if event.type == py.VIDEORESIZE:
            win = display.set_mode((event.w, event.h), py.RESIZABLE)
            #print(event.w)
            #print(event.h)

        if event.type == py.KEYDOWN:
            if event.key == py.K_UP:
                z_move = True
                z_scale = 1
            elif event.key == py.K_DOWN:
                z_move = True
                z_scale = -1
            elif event.key == py.K_LEFT:
                x_move = True
                x_scale = -1
            elif event.key == py.K_RIGHT:
                x_move = True
                x_scale = 1
            elif event.key == py.K_RETURN:
                a_move = True
                a_scale = 1
            elif event.key == py.K_LSHIFT:
                a_move = True
                a_scale = -1
            elif event.key == py.K_r:
                if record is False:
                    record = True
                    file_number += 1
                    file_name = "Data"+str(file_number)+".txt"
                    start_time = 0
                    delta = 0
                    file = open(file_name, "w")
                    file.write("t\tx\tz\ta\ttheta\tphi\tgamma\n")
                else:
                    record = False
                    file.close()

        if event.type == py.KEYUP:
            if event.key == py.K_UP:
                z_move = False
            elif event.key == py.K_DOWN:
                z_move = False
            elif event.key == py.K_LEFT:
                x_move = False
            elif event.key == py.K_RIGHT:
                x_move = False
            elif event.key == py.K_RETURN:
                a_move = False
            elif event.key == py.K_LSHIFT:
                a_move = False

    # move claw
    if x_move is True:
        Claw_Pos[0] += 0.01*x_scale
    if z_move is True:
        Claw_Pos[1] += 0.01*z_scale
    if a_move is True:
        alpha += 0.01*a_scale


    # calculate angles
    u = Claw_Pos[0] - Claw_L * m.cos(alpha)
    w = Claw_Pos[1] - Claw_L * m.sin(alpha)
    B = m.sqrt(m.pow(u, 2) + m.pow(w, 2))
    b = m.acos((m.pow(Upper_L, 2) + m.pow(Lower_L, 2) - m.pow(B, 2)) / (2 * Upper_L * Lower_L))
    a = m.asin(Upper_L * m.sin(b) / B)
    c = m.asin(Lower_L * m.sin(b) / B)
    theta = a + m.atan(w / u)
    phi = m.pi/2 - b
    gamma = alpha + c + m.atan(u / w) - m.pi/2

    # calculate joint positions
    Wrist_Pos = num.array([u, w])
    Elbow_Pos = num.array([Lower_L*m.cos(theta), Lower_L*m.sin(theta)])

    # draw lines
    Shoulder_Coord = [1500, 800]-Shoulder_Pos*m_to_pixels-offset
    Elbow_Coord = [1500, 800]-Elbow_Pos*m_to_pixels-offset
    Wrist_Coord = [1500, 800]-Wrist_Pos*m_to_pixels-offset
    Claw_Coord = [1500, 800]-Claw_Pos*m_to_pixels-offset

    if Lower_L-Lower_L*0.01 < m.sqrt(m.pow(Shoulder_Pos[0]-Elbow_Pos[0], 2) + m.pow(Shoulder_Pos[1]-Elbow_Pos[1], 2)) < Lower_L+Lower_L*0.01:
        Lower_C = [0, 255, 255]
    else:
        Lower_C = [255, 0, 0]

    if Upper_L-Upper_L*0.01 < m.sqrt(m.pow(Elbow_Pos[0]-Wrist_Pos[0], 2) + m.pow(Elbow_Pos[1]-Wrist_Pos[1], 2)) < Upper_L+Upper_L*0.01:
        Upper_C = [255, 0, 255]
    else:
        Upper_C = [255, 0, 0]

    if Claw_L-Claw_L*0.01 < m.sqrt(m.pow(Wrist_Pos[0]-Claw_Pos[0], 2) + m.pow(Wrist_Pos[1]-Claw_Pos[1], 2)) < Claw_L+Claw_L*0.01:
        Claw_C = [255, 255, 0]
    else:
        Claw_C = [255, 0, 0]

    draw.line(win, Lower_C, Shoulder_Coord, Elbow_Coord, 5)
    draw.line(win, Upper_C, Elbow_Coord, Wrist_Coord, 5)
    draw.line(win, Claw_C, Wrist_Coord, Claw_Coord, 5)

    if record is True:
        delta += clock.get_time()
        t = start_time+delta
        file.write(
            "{0:.0f}\t{1:.3f}\t{2:.3f}\t{3:.3f}\t{4:.3f}\t{5:.3f}\t{6:.3f}\n".format(t, Claw_Pos[0], Claw_Pos[1], alpha, theta,
                                                                                     phi, gamma))

    py.transform.rotate(win, 180)
    display.flip()
    win.fill((0, 0, 0))
    clock.tick(30)

py.quit()