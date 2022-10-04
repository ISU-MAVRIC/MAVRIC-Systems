# install modules
import pygame as py
import pygame.event as event
import pygame.key as key
import pygame.display as display
import pygame.draw as draw
import pygame.mouse as mouse
import pygame.joystick as joystick
import socket
import math

# install classes
import Base_Data as Base
import scarab

# initial values
joysticks = []
drive_stick_name = "Mad Catz V.1 Stick"
arm_stick_name = "Controller (XBOX 360 For Windows)"
master_ip = "192.168.1.10"
steer = 0


# function for setting wheel drive and steer
def set_drive(tank, car, point, tdrive, tsteer, cdrive, csteer, pdrive, psteer):
    if tank:
        left = -100*(tdrive - tsteer/2)
        right = -100*(tdrive + tsteer/2)
        rover.set_wheels(left, right, 0, 0)
    elif car:
        steer = 100*csteer
        left = -100*cdrive
        right = -100*cdrive
        if steer >= 50:
            steer = 50
        if steer <= -50:
            steer = -50
        strleft = steer
        strright = -steer
        rover.set_wheels(left, right, strleft, strright)
    elif point:
        left = 100*pdrive
        right = -100*pdrive
        strleft = psteer
        strright = psteer
        rover.set_wheels(left, right, strleft, strright)


# initiate elements
drive_xy = Base.Axis([0, 0], 100, 100, "rectangle", "Drive")
drive_throttle = Base.Axis([0], 225, 250, "vertical", "Drive Throttle")
drive_rot = Base.Axis([0], 100, 250, "horizontal", "Point Steer")
arm_xyl = Base.Axis([0, 0], 350, 100, "rectangle", "Shoulder Vel")
arm_xyr = Base.Axis([0, 0], 450, 100, "rectangle", "Wrist Vel")
arm_trigl = Base.Axis([1], 333, 250, "vertical", "Elbow Retract Vel")
arm_trigr = Base.Axis([1], 450, 250, "vertical", "Elbow Extend Vel")
arm_bumper = Base.Button([0, 0], 225, 75, "column", ["Claw Open", "Claw Close"])
point_steer = Base.Button([0, 0, 0, 0], 150, 450, "cross", ["Str 0", "Dri 0", "Rot Neg", "Rot Pos", "P Str"])
steer_option = Base.Button([0, 0, 0], 75, 625, "row", ["Tank Drive", "Car Drive", "Point Steer"])
arm_cross = Base.Button([0, 0, 0, 0], 400, 450, "cross", ["+X Pos", "-X Pos", "+Y Pos", "-Y Pos", "Arm P"])
arm_ab = Base.Button([0, 0], 363, 625, "row", ["-Z Pos", "+Z Pos"])
steer_cal = Base.Button([0], 100, 725, "row", ["Steer Cal"])
arm_cal = Base.Button([0], 400, 725, "row", ["Arm Cal"])
auto_en = Base.Button([0, 0], 200, 725, "row", ["Auto Enable", "Auto Disable"])
temp_data = Base.Data("Temperature", 618, 625, "C")
bat1_data = Base.Data("Bat 1 Voltage", 768, 625, "V")
bat2_data = Base.Data("Bat 2 Voltage", 918, 625, "V")
long_data = Base.Data("Longitude", 618, 525, "deg")
lat_data = Base.Data("Latitude", 768, 525, "deg")
head_data = Base.Data("Heading", 918, 525, "deg")
gps_map = Base.Map(1536/2, 250)

# py initialization
py.init()
joystick.init()
win = display.set_mode((500, 500), py.RESIZABLE)
display.set_caption('MAVRIC Base Station')
infofont = py.font.SysFont("Arial", 15)
titlefont = py.font.SysFont("Arial", 15)
northfont = py.font.SysFont("Arial", 18)


# find and init controllers
for i in range(0, joystick.get_count()):
    joysticks.append(joystick.Joystick(i))
    joysticks[-1].init()

    # select drive stick based on name
    #   this means using a different stick will require the hardcoded name to change
    if joysticks[-1].get_name() == drive_stick_name:
        print("Detected drive joystick '%s'" % joysticks[-1].get_name())
        drive_joy = Base.Joysticks(joysticks[-1])
    # select drive controller based on name
    #   this means using a different controller will require the hardcoded name to change
    elif joysticks[-1].get_name() == arm_stick_name:
        print("Detected arm controller '%s'" % joysticks[-1].get_name())
        arm_joy = Base.Joysticks(joysticks[-1])

    else:
        print("Detected unbound controller '%s'" % joysticks[-1].get_name())

# initialize rover
rover = scarab.Scarab(master_ip)
rover.open()
rover.kill_all()

run = True
while run:
    # process game events
    for event in py.event.get():
        # if red 'X' is clicked, quit program
        if event.type == py.QUIT:
            run = False
        # if window is resized
        if event.type == py.VIDEORESIZE:
            win = display.set_mode((event.w, event.h), py.RESIZABLE)
            print(event.w)
            print(event.h)

        # handle keyboard events
        if event.type == py.KEYDOWN:
            # software emergency stop
            if event.key == py.K_SPACE:
                rover.kill_all()
                emergency_stop = True
                autonomous_enabled = False

            # software emergency stop reset
            elif event.key == py.K_RETURN:
                emergency_stop = False

        # joy axis motion
        if event.type == py.JOYAXISMOTION:
            try:
                if drive_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.axis == 0:
                        drive_xy.axis[0] = round(event.value, 2)
                    elif event.axis == 1:
                        drive_xy.axis[1] = round(event.value, 2)
                    elif event.axis == 2:
                        drive_throttle.axis[0] = round(event.value, 2)
                    elif event.axis == 3:
                        drive_rot.axis[0] = round(event.value, 2)
                elif arm_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.axis == 0:
                        arm_xyl.axis[0] = round(event.value, 2)
                    if event.axis == 1:
                        arm_xyl.axis[1] = round(event.value, 2)
                    if event.axis == 2:
                        if event.value > 0:
                            value = -2*(abs(event.value)-0.5)
                            arm_trigl.axis[0] = round(value, 2)
                        elif event.value < 0:
                            value = -2*(abs(event.value)-0.5)
                            arm_trigr.axis[0] = round(value, 2)
                        else:
                            arm_trigl.axis[0] = 1
                            arm_trigr.axis[0] = 1
                    if event.axis == 3:
                        arm_xyr.axis[1] = round(event.value, 2)
                    if event.axis == 4:
                        arm_xyr.axis[0] = round(event.value, 2)
            except:
                print("uh oh")
                pass
        # joy hat buttons
        if event.type == py.JOYHATMOTION:
            try:
                if drive_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.hat == 0:
                        if event.value[1] == 1:
                            point_steer.buttons[0] = 1
                            point_steer.buttons[1] = 0
                            point_steer.buttons[2] = 0
                            point_steer.buttons[3] = 0
                            steer = 0
                        if event.value[1] == -1:
                            point_steer.buttons[0] = 0
                            point_steer.buttons[1] = 1
                            point_steer.buttons[2] = 0
                            point_steer.buttons[3] = 0
                        if event.value[0] == 1:
                            point_steer.buttons[0] = 0
                            point_steer.buttons[1] = 0
                            point_steer.buttons[2] = 0
                            point_steer.buttons[3] = 1
                            steer = 100
                        if event.value[0] == -1:
                            point_steer.buttons[0] = 0
                            point_steer.buttons[1] = 0
                            point_steer.buttons[2] = 1
                            point_steer.buttons[3] = 0
                            steer = -100
            except:
                print("uh oh")
                pass

        # Joy buttons
        if event.type == py.JOYBUTTONDOWN:
            try:
                if drive_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.button == 4:
                        steer_option.buttons[0] = 1
                        steer_option.buttons[1] = 0
                        steer_option.buttons[2] = 0
                    if event.button == 1:
                        steer_option.buttons[0] = 0
                        steer_option.buttons[1] = 1
                        steer_option.buttons[2] = 0
                    if event.button == 5:
                        steer_option.buttons[0] = 0
                        steer_option.buttons[1] = 0
                        steer_option.buttons[2] = 1
                    if event.button == 0:
                        rover.kill_all()
                    if event.button == 2:
                        rover.enable_autonomous()
                        auto_en.buttons[0] = 1
                        auto_en.buttons[1] = 0
                    if event.button == 3:
                        rover.disable_autonomous()
                        auto_en.buttons[0] = 0
                        auto_en.buttons[1] = 1

            except:
                print("uh oh")
                pass

    # set wheel drive and position
    set_drive(steer_option.buttons[0], steer_option.buttons[1], steer_option.buttons[2], drive_xy.axis[1],
    drive_xy.axis[0], drive_xy.axis[1], drive_rot.axis[0], drive_rot.axis[0], steer)

    # draw axis in center of screen
    drive_xy.draw(win, infofont, (255, 255, 255), (0, 255, 0))
    drive_throttle.draw(win, infofont, (255, 255, 255), (255, 255, 255))
    drive_rot.draw(win, infofont, (255, 255, 255), (0, 0, 255))
    arm_xyl.draw(win, infofont, (255, 255, 255), (255, 0, 0))
    arm_xyr.draw(win, infofont, (255, 255, 255), (255, 0, 0))
    arm_trigl.draw(win, infofont, (255, 255, 255), (255, 255, 0))
    arm_trigr.draw(win, infofont, (255, 255, 255), (255, 255, 0))
    arm_bumper.draw(win, infofont, (255, 255, 255), (255, 255, 0))
    point_steer.draw(win, infofont, (255, 255, 255), (21, 25, 101))
    steer_option.draw(win, infofont, (255, 255, 255), (145, 199, 136))
    arm_cross.draw(win, infofont, (255, 255, 255), (240, 89, 69))
    arm_ab.draw(win, infofont, (255, 255, 255), (240, 89, 69))
    steer_cal.draw(win, infofont, (255, 255, 255), (40, 150, 114))
    arm_cal.draw(win, infofont, (255, 255, 255), (129, 0, 0))
    auto_en.draw(win, infofont, (255, 255, 255), (105, 48, 195))
    temp_data.draw(win, rover.temperature, infofont, (255, 255, 255), (200, 0, 200))
    bat1_data.draw(win, rover.voltage[0], infofont, (255, 255, 255), (232, 69, 69))
    bat2_data.draw(win, rover.voltage[1], infofont, (255, 255, 255), (232, 69, 69))
    long_data.draw(win, rover.gps.longitude, infofont, (255, 255, 255), (91, 138, 114))
    lat_data.draw(win, rover.gps.longitude, infofont, (255, 255, 255), (91, 138, 114))
    head_data.draw(win, rover.gps.heading, infofont, (255, 255, 255), (91, 138, 114))
    gps_map.draw_map(win, northfont, (0, 255, 0), (100, 0, 100))
    gps_map.draw_rover(win, gps_map.x+gps_map.base/2, gps_map.y+gps_map.height/2)

    display.flip()
    py.time.delay(10)
    win.fill((0, 0, 0))

# end
py.quit()
rover.kill_all()
rover.close()
