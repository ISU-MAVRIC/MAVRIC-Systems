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
from csv import reader

# install classes
import Base_Data as Base
import scarab

# initial values
joysticks = []
drive_stick_name =  "Madcatz Mad Catz V.1 Stick" #"Logitech Extreme 3D"
drive_joy_name = " " #"Controller (Xbox One For Windows)"
arm_stick_name =  "PDP Xbox 360 Afterglow" #'Xbox One S Controller' #Xbox 360 Controller" # 'Xbox One S Controller'"PDP Xbox 360 Afterglow"
joy = True
master_ip = "192.168.1.10"
cal = [0, 0, 0, 0]
cont_str = 0
m_drive = 0
m_steer = 0
d_lf = 0
d_lm = 0
d_lb = 0
d_rf = 0
d_rm = 0
d_rb = 0
str_lf = 0
str_lb = 0
str_rf = 0
str_rb = 0
lumin_no = 0
arm_cl_open = False
arm_cl_close = False
sr_vel = 0
sp_vel = 0
ep_vel = 0
wr_vel = 0
wp_vel = 0
cl_control = 0
cl_pos = [0,0,0]
cl_delta = [0,0,0]
alpha = 0
wheel_length = 37.5
wheel_width = 28.5
temp = None
voltage = [None, None]
mouse_pos = [0, 0]
ac_hor = 0
ac_vert = 0
hook = 1

# colors
grey = (82, 82, 82)
light_grey = (126, 116, 116)
dark_grey = (34, 40, 49)
blue = (51, 47, 208)
dark_blue = (14, 24, 95)
red = (218, 0, 55)
light_red = (255, 0, 0)
dark_red = (132, 20, 45)
white = (255, 255, 255)
black = (0, 0, 0)
green = (119, 217, 112)
light_green = (78, 159, 61)
yellow = (255, 211, 105)
gold = (216, 146, 22)
orange = (249, 178, 8)
light_orange = (255, 159, 69)
cyan = (110, 220, 217)
light_blue = (125, 237, 255)
purple = (146, 84, 200)
magenta = (225, 95, 237)
pink = (250, 88, 182)


# function for calculating drive, and steer motor speed values
# returns d_lf, d_lm, d_lb, d_rf, d_rm, d_rb, str_lf, str_lb, str_rf, str_rb
def calc_drive(tank, car, point, manual, tdrive, tsteer, cdrive, csteer, pdrive, motors, mdrive, msteer,
               sensdrive, senssteer):
    sensdrive = -sensdrive*0.5+.5
    senssteer = -senssteer*0.5+.5
    if tank:
        left = -100*(tdrive*sensdrive - tsteer*senssteer/2)
        right = -100*(tdrive*sensdrive + tsteer*senssteer/2)
        #print(sensdrive)
        return left, left, left, right, right, right, 0, 0, 0, 0
    elif car:
        in_angle = math.radians(abs(csteer*100*senssteer))*0.9
        #print(in_angle)
        if in_angle != 0:
            out_angle = math.pi/2-math.atan(1/math.tan(in_angle)+2*wheel_width/wheel_length)
            in_r = wheel_length / (2 * math.sin(in_angle))
            out_r = wheel_length / (2 * math.sin(out_angle))
            center_r = in_r * math.cos(in_angle) + wheel_width / 2
            in_mid_r = center_r - wheel_width / 2
            out_mid_r = center_r + wheel_width / 2
            out_v = -100*cdrive * sensdrive
            angle_v = out_v / out_r
            in_v = in_r * angle_v
            in_mid_v = in_mid_r * angle_v
            out_mid_v = out_mid_r * angle_v
        else:
            out_angle = 0
            in_v = 0
            in_mid_v = 0
            out_v = 0
            out_mid_v = 0

        if csteer < 0:
            return in_v, in_mid_v, in_v, out_v, out_mid_v, out_v, math.degrees(in_angle), math.degrees(in_angle), \
                   math.degrees(out_angle), math.degrees(out_angle)
        elif csteer > 0:
            return out_v, out_mid_v, out_v, in_v, in_mid_v, in_v, -math.degrees(out_angle), -math.degrees(out_angle), \
                   -math.degrees(in_angle), -math.degrees(in_angle)
        else:
            return -cdrive*sensdrive*100, -cdrive*sensdrive*100, -cdrive*sensdrive*100, -cdrive*sensdrive*100, \
                   -cdrive*sensdrive*100, -cdrive*sensdrive*100, 0, 0, 0, 0
    elif point:
        str_angle = math.degrees(math.atan(wheel_length/wheel_width))
        str_r = math.sqrt(math.pow(wheel_width/2, 2)+math.pow(wheel_length/2, 2))
        mid_r = wheel_width/2
        str_v = abs(pdrive*sensdrive*100)
        mid_v = str_v*mid_r/str_r
        if pdrive < 0:
            return -str_v, -mid_v, -str_v, str_v, mid_v, str_v, -str_angle, -str_angle, str_angle, str_angle
        elif pdrive > 0:
            return str_v, mid_v, str_v, -str_v, -mid_v, -str_v, -str_angle, -str_angle, str_angle, str_angle
        else:
            return 0, 0, 0, 0, 0, 0, -str_angle, -str_angle, str_angle, str_angle
    elif manual:
        m_v = mdrive*sensdrive*100
        #print(m_v)
        s_v = msteer*senssteer*100
        return -m_v*motors[0], -m_v*motors[1], -m_v*motors[2], -m_v*motors[3], -m_v*motors[4], -m_v*motors[5], \
               s_v*motors[6], s_v*motors[7], s_v*motors[8], s_v*motors[9]
    else:
        return 0, 0, 0, 0, 0, 0, 0, 0, 0, 0


"""
# function for setting wheel drive and steer
def set_drive(tank, car, point, tdrive, tsteer, cdrive, csteer, pdrive, psteer):
    if tank:
        left = -100*(tdrive - tsteer/2)
        right = -100*(tdrive + tsteer/2)
        rover.set_drive(left, right, 0, 0)
        rover.set_steer(0, 0, 0, 0)
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
        rover.set_drive(left, right, 0, 0)
        rover.set_steer(strleft, strleft, strright, strright)
    elif point:
        left = 100*pdrive
        right = -100*pdrive
        strleft = psteer*0.7
        strright = psteer*0.7
        rover.set_drive(left, right, 0, 0)
        rover.set_steer(strleft, strleft, strright, strright)


# function for manually control the steer wheels and calibrating them
def set_steer(manual, cal, lf, lb, rf, rb, cal_lf, cal_lb, cal_rf, cal_rb):
    if manual:
        rover.set_steer(lf*100, lb*100, rf*100, rb*100)
    elif cal:
        rover.set_cal(cal_lf, cal_lb, cal_rf, cal_rb)
"""


# function for setting arm motor velocities
def set_arm(sr, sp, ep, wr, wp, open, close, cl_pos):
    if -0.2 < ep < 0.2:
        ep = 0
    if -0.1 < sp < 0.1:
        sp = 0
    if -0.1 < sr < 0.1:
        sr = 0
    if -0.2 < wr < 0.2:
        wr = 0
    if -0.1 < wp < 0.1:
        wp = 0
    #rover.set_arm_base_rot(sr*100)
    #rover.set_arm_base_pitch(-sp*100)
    #rover.set_arm_elbow_pitch(ep*100)
    #rover.set_arm_claw_rot(-wr*100)
    #rover.set_arm_claw_pitch(wp*100)
    rover.set_arm_all(sr*100, -sp*100, ep*100, -wr*100, wp*100)
    if open is True:
        cl_pos = cl_pos + 5
    elif close is True:
        cl_pos = cl_pos - 5
    if cl_pos > 100:
        cl_pos = 100
    elif cl_pos < -100:
        cl_pos = -100
    rover.set_arm_claw_actuation(cl_pos)
    return cl_pos


# function for calculating arm motor positions
def calc_arm(x, y, z, alpha, beta, open, close, cl_pos):
    if -0.5 < x < 0.5:
        x = 0
    if -0.5 < y < 0.5:
        y = 0
    if -0.5 < z < 0.5:
        z = 0
    if -0.5 < alpha < 0.5:
        alpha = 0
    if -0.5 < beta < 0.5:
        beta = 0
    scale = 0.003
    delta_x = x*scale
    delta_y = y*-scale
    delta_z = z*-scale
    delta_alpha = alpha*scale
    delta_beta = beta*scale
    if open is True:
        cl_pos = cl_pos + 1
    elif close is True:
        cl_pos = cl_pos - 1
    rover.set_arm_pos(round(delta_x, 4), round(delta_y, 4), round(delta_z, 4), round(delta_alpha, 4), round(delta_beta, 4), round(cl_pos, 4))
    return cl_pos


# initiate elements
drive_xy = Base.Axis([0, 0], 100, 100, "rectangle", "Drive")
drive_cont_xy = Base.Axis([0, 0], 100, 100, "circle", "Drive")
drive_sens = Base.Axis([0], 225, 250, "vertical", "Drive Sensitivity")
drive_cont_throttle = Base.Axis([0], 225, 250, "vertical", "Drive Throttle")
drive_rot = Base.Axis([0], 100, 250, "horizontal", "Point Steer")
drive_cont_left = Base.Axis([1], 50, 250, "vertical", "Left")
drive_cont_right = Base.Axis([1], 150, 250, "vertical", "Right")
arm_xyl = Base.Axis([0, 0], 350, 100, "rectangle", "Shoulder Vel")
arm_xyr = Base.Axis([0, 0], 450, 100, "rectangle", "Wrist Vel")
arm_trigl = Base.Axis([1], 333, 250, "vertical", "Elbow Retract Vel")
arm_trigr = Base.Axis([1], 450, 250, "vertical", "Elbow Extend Vel")
arm_sr_scale = Base.Axis([-1], 1050, 625, "vertical", "SR Scale")
arm_sp_scale = Base.Axis([-1], 1150, 625, "vertical", "SP Scale")
arm_ep_scale = Base.Axis([-1], 1250, 625, "vertical", "EP Scale")
arm_wr_scale = Base.Axis([-1], 1350, 625, "vertical", "WR Scale")
arm_wp_scale = Base.Axis([-1], 1450, 625, "vertical", "WP Scale")
arm_scale_control = Base.Button([0, 0, 0, 0, 0], 1150, 750, "row", ["SR S", "SP S", "EP S", "WR S", "WP S"])
arm_bumper = Base.Button([0, 0], 225, 75, "column", ["Claw Open", "Claw Close"])
point_steer = Base.Button([0, 0, 0, 0], 150, 450, "cross", ["Str 0", "Dri 0", "Rot Neg", "Rot Pos", "P Str"])
steer_option = Base.Button([0, 0, 0, 0], 100, 375, "column", ["Tank Drive", "Car Drive", "Point Steer", "Calibration"])
arm_cross = Base.Button([0, 0, 0, 0], 400, 450, "cross", ["+X Pos", "-X Pos", "+Y Pos", "-Y Pos", "Arm P"])
arm_cam_control = Base.Axis([0, 0], 400, 450, "rectangle", "Arm Cam")
util_control = Base.Button([0, 0], 300, 600, "column", ["Util Pos", "Util Neg"])
hook_control = Base.Button([0, 0], 400, 600, "column", ["Hook Pos", "Hook Neg"])
sci_control = Base.Button([0, 0], 500, 600, "column", ["Sci Pos", "Sci Neg"])
lumin_control = Base.Button([0], 200, 600, "column", ["L Button"])
arm_ab = Base.Button([0, 0], 363, 625, "row", ["-Z Pos", "+Z Pos"])
steer_cal = Base.Button([0, 0, 0, 0], 825, 750, "row", ["Lf Cal", "Lb Cal", "Rf Cal", "Rb Cal"])
manual_control = Base.Button([0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 50, 750, "row", ["Lf D", "Lm D", "Lb D", "Rf D", "Rm D"
                             , "Rb D", "Lf S", "Lb S", "Rf S", "Rb S"])
drive_manual_xy = Base.Axis([0, 0], 100, 625, "rectangle", "Manual")
arm_enable = Base.Button([0, 1], 225, 525, "column", ["Arm Enable", "Arm Disable"])
auto_en = Base.Button([0, 1], 225, 375, "column", ["Auto Enable", "Auto Disable"])
temp_data = Base.Data("Temperature", 618, 625, 100, "C", 2)
bat1_data = Base.Data("Bat 1 Voltage", 768, 625, 100, "V", 2)
bat2_data = Base.Data("Bat 2 Voltage", 918, 625, 100, "V", 2)
long_data = Base.Data("Longitude", 618, 525, 125, "deg", 6)
lat_data = Base.Data("Latitude", 768, 525, 125, "deg", 6)
head_data = Base.Data("Heading", 918, 525, 100, "deg", 2)
#gps_map = Base.Map(1536/2, 250, "maps/ISU_42.0358_-93.6558_42.0226_-93.6331.png", [-93.6558, 42.0358, -93.6331, 42.0226]) #"maps/ISU_42.0358_-93.6558_42.0226_-93.6331.png"


# py initialization
py.init()
joystick.init()
win = display.set_mode((500, 500), py.RESIZABLE)
display.set_caption('MAVRIC Base Station')
infofont = py.font.SysFont("Arial", 15)
titlefont = py.font.SysFont("Arial", 15)
northfont = py.font.SysFont("Arial", 18)
gpsfont = py.font.SysFont("Arial", 14)

wp_map = Base.Map(600, 50, 400, 400, "map", infofont, white, win, dark_red)
wp_list = Base.Waypoint(1050, 50, 240, 400, gpsfont, white, black, win, white, [red, green, blue, yellow, orange, pink], light_blue)


# find and init controllers
for i in range(0, joystick.get_count()):
    joysticks.append(joystick.Joystick(i))
    joysticks[-1].init()
    print(joysticks[-1].get_name())

    # select drive stick based on name
    #   this means using a different stick will require the hardcoded name to change
    if joysticks[-1].get_name() == drive_stick_name:
        print("Detected drive joystick '%s'" % joysticks[-1].get_name())
        drive_joy = Base.Joysticks(joysticks[-1])
        joy = True
    # select drive controller based on name
    #   this means using a different stick will require the hardcoded name to change
    elif joysticks[-1].get_name() == drive_joy_name:
        print("Detected drive controller '%s'" % joysticks[-1].get_name())
        drive_cont = Base.Joysticks(joysticks[-1])
        joy = False
    # select RM controller based on name
    #   this means using a different controller will require the hardcoded name to change
    elif joysticks[-1].get_name() == arm_stick_name:
        print("Detected arm controller '%s'" % joysticks[-1].get_name())
        arm_joy = Base.Joysticks(joysticks[-1])

    else:
        print("Detected unbound controller '%s'" % joysticks[-1].get_name())

# initialize rover
rover = scarab.Scarab(master_ip, False)
rover.open()
rover.kill_all()

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

        # handle keyboard events
        if event.type == py.KEYDOWN:
            # software emergency stop
            if event.key == py.K_SPACE:
                rover.kill_all()
                emergency_stop = True
                autonomous_enabled = False

            # auto enable/disable
            elif event.key == py.K_e:
                rover.enable_autonomous()
                auto_en.buttons[0] = 1
                auto_en.buttons[1] = 0
            elif event.key == py.K_d:
                rover.disable_autonomous()
                auto_en.buttons[0] = 0
                auto_en.buttons[1] = 1
            elif event.key == py.K_DELETE:
                wp_list.remove = True
                #gps_map.forget_waypoints(rover)

            elif event.key == py.K_o:
                rover.set_arm_enable()
                arm_enable.buttons[0] = 1
                arm_enable.buttons[1] = 0

            elif event.key == py.K_l:
                rover.set_arm_disable()
                arm_enable.buttons[0] = 0
                arm_enable.buttons[1] = 1

            # software emergency stop reset
            elif event.key == py.K_RETURN:
                emergency_stop = False

            # add waypoints from csv
            elif event.key == py.K_w:
                #wp_list.add = True
                with open('WP.csv', 'r') as waypoints:
                    waypoint_list = reader(waypoints)
                    for wp in waypoint_list:
                        rover.add_waypoint(wp[0], wp[1], wp[2])

            elif event.key == py.K_q:
                wp_list.down = True
            elif event.key == py.K_a:
                wp_list.up = True

            # steer cal
            elif event.key == py.K_1:
                steer_cal.buttons[0] = 1
                drive_manual_xy.axis[0] = 0
            elif event.key == py.K_2:
                steer_cal.buttons[1] = 1
                drive_manual_xy.axis[0] = 0
            elif event.key == py.K_3:
                steer_cal.buttons[2] = 1
                drive_manual_xy.axis[0] = 0
            elif event.key == py.K_4:
                steer_cal.buttons[3] = 1
                drive_manual_xy.axis[0] = 0

            # arm scale options
            elif event.key == py.K_6:
                if arm_scale_control.buttons[0] == 0:
                    arm_scale_control.buttons[0] = 1
                else:
                    arm_scale_control.buttons[0] = 0
            elif event.key == py.K_7:
                if arm_scale_control.buttons[1] == 0:
                    arm_scale_control.buttons[1] = 1
                else:
                    arm_scale_control.buttons[1] = 0
            elif event.key == py.K_8:
                if arm_scale_control.buttons[2] == 0:
                    arm_scale_control.buttons[2] = 1
                else:
                    arm_scale_control.buttons[2] = 0
            elif event.key == py.K_9:
                if arm_scale_control.buttons[3] == 0:
                    arm_scale_control.buttons[3] = 1
                else:
                    arm_scale_control.buttons[3] = 0
            elif event.key == py.K_0:
                if arm_scale_control.buttons[4] == 0:
                    arm_scale_control.buttons[4] = 1
                else:
                    arm_scale_control.buttons[4] = 0

            # util control
            elif event.key == py.K_PAGEUP:
                util_control.buttons[0] = 1
                #hook_control.buttons[0] = 1

            elif event.key == py.K_PAGEDOWN:
                util_control.buttons[1] = 1
                #hook_control.buttons[1] = 1

            # hook control
            elif event.key == py.K_F10:
                hook_control.buttons[0] = 1

            elif event.key == py.K_F11:
                hook_control.buttons[1] = 1

            # sci control
            elif event.key == py.K_F8:
                sci_control.buttons[0] = 1

            elif event.key == py.K_F9:
                sci_control.buttons[1] = 1

            elif event.key == py.K_i:
                lumin_control.buttons[0] = 1

            elif event.key == py.K_i:
                lumin_no = 1

            # manual drive select
            elif event.key == py.K_KP_0:
                if manual_control.buttons[0] == 0:
                    manual_control.buttons[0] = 1
                else:
                    manual_control.buttons[0] = 0
            elif event.key == py.K_KP_1:
                if manual_control.buttons[1] == 0:
                    manual_control.buttons[1] = 1
                else:
                    manual_control.buttons[1] = 0
            elif event.key == py.K_KP_2:
                if manual_control.buttons[2] == 0:
                    manual_control.buttons[2] = 1
                else:
                    manual_control.buttons[2] = 0
            elif event.key == py.K_KP_3:
                if manual_control.buttons[3] == 0:
                    manual_control.buttons[3] = 1
                else:
                    manual_control.buttons[3] = 0
            elif event.key == py.K_KP_4:
                if manual_control.buttons[4] == 0:
                    manual_control.buttons[4] = 1
                else:
                    manual_control.buttons[4] = 0
            elif event.key == py.K_KP_5:
                if manual_control.buttons[5] == 0:
                    manual_control.buttons[5] = 1
                else:
                    manual_control.buttons[5] = 0
            elif event.key == py.K_KP_6:
                if manual_control.buttons[6] == 0:
                    manual_control.buttons[6] = 1
                else:
                    manual_control.buttons[6] = 0
            elif event.key == py.K_KP_7:
                if manual_control.buttons[7] == 0:
                    manual_control.buttons[7] = 1
                else:
                    manual_control.buttons[7] = 0
            elif event.key == py.K_KP_8:
                if manual_control.buttons[8] == 0:
                    manual_control.buttons[8] = 1
                else:
                    manual_control.buttons[8] = 0
            elif event.key == py.K_KP_9:
                if manual_control.buttons[9] == 0:
                    manual_control.buttons[9] = 1
                else:
                    manual_control.buttons[9] = 0

            # manual drive and steer
            elif event.key == py.K_UP:
                m_drive += 1
            elif event.key == py.K_DOWN:
                m_drive += -1
            elif event.key == py.K_LEFT:
                m_steer += -1
            elif event.key == py.K_RIGHT:
                m_steer += 1
            elif event.key == py.K_KP_PERIOD:
                drive_manual_xy.axis[1] = 0
            elif event.key == py.K_KP_ENTER:
                drive_manual_xy.axis[0] = 0

        if event.type == py.KEYUP:

            # steer cal
            if event.key == py.K_1:
                steer_cal.buttons[0] = 0
            elif event.key == py.K_2:
                steer_cal.buttons[1] = 0
            elif event.key == py.K_3:
                steer_cal.buttons[2] = 0
            elif event.key == py.K_4:
                steer_cal.buttons[3] = 0

            # manual drive and steer
            elif event.key == py.K_UP:
                m_drive = 0
            elif event.key == py.K_DOWN:
                m_drive = 0
            elif event.key == py.K_LEFT:
                m_steer = 0
            elif event.key == py.K_RIGHT:
                m_steer = 0

            # util control
            elif event.key == py.K_PAGEUP:
                util_control.buttons[0] = 0
                #hook_control.buttons[0] = 0

            # hook control
            elif event.key == py.K_PAGEDOWN:
                util_control.buttons[1] = 0
                #hook_control.buttons[1] = 0

            # hook control
            elif event.key == py.K_F10:
                hook_control.buttons[0] = 0

            elif event.key == py.K_F11:
                hook_control.buttons[1] = 0

            # sci control
            elif event.key == py.K_F8:
                sci_control.buttons[0] = 0

            elif event.key == py.K_F9:
                sci_control.buttons[1] = 0

            elif event.key == py.K_i:
                lumin_control.buttons[0] = 0

            elif event.key == py.K_i:
                lumin_no = 0


        if event.type == py.MOUSEBUTTONDOWN:
            # map controls
            """
            if gps_map.x < event.pos[0] < gps_map.x+gps_map.base and gps_map.y < event.pos[1] < gps_map.y+gps_map.height:
                if event.button == 4:
                    gps_map.change_map(event.pos, -1)
                elif event.button == 5:
                    gps_map.change_map(event.pos, 1)
                elif event.button == 1:
                    gps_map.change_map(event.pos, 0)
                elif event.button == 3:
                    gps_map.add_waypoint(rover, event.pos)
            """

        if event.type == py.MOUSEMOTION:
            # get mouse position
            mouse_pos = event.pos

        # joy axis motion
        if event.type == py.JOYAXISMOTION:
            try:
                if drive_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.axis == 0:
                        drive_xy.axis[0] = round(event.value, 2)
                    elif event.axis == 1:
                        drive_xy.axis[1] = round(event.value, 2)
                    elif event.axis == 2:
                        if arm_scale_control.buttons[0] == 1:
                            arm_sr_scale.axis[0] = round(event.value, 2)
                            print(event.value)
                        elif arm_scale_control.buttons[1] == 1:
                            arm_sp_scale.axis[0] = round(event.value, 2)
                        elif arm_scale_control.buttons[2] == 1:
                            arm_ep_scale.axis[0] = round(event.value, 2)
                        elif arm_scale_control.buttons[3] == 1:
                            arm_wr_scale.axis[0] = round(event.value, 2)
                        elif arm_scale_control.buttons[4] == 1:
                            arm_wp_scale.axis[0] = round(event.value, 2)
                        else:
                            drive_sens.axis[0] = round(event.value, 2)
                    elif event.axis == 3:
                        drive_rot.axis[0] = round(event.value, 2)
            except:
                print("no drive joy")
                pass
            try:
                if arm_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.axis == 0:
                        arm_xyl.axis[0] = -1 * round(event.value, 2)
                    if event.axis == 1:
                        arm_xyl.axis[1] = -1 * round(event.value, 2)
                    if event.axis == 5:
                        ep_vel = (round(event.value) + 1) / 2
                        arm_trigl.axis[0] = -round(event.value, 2)
                    if event.axis == 2:
                        ep_vel = -(round(event.value) + 1) / 2
                        arm_trigr.axis[0] = -round(event.value, 2)
                    if event.axis == 3:
                        arm_xyr.axis[0] =  round(event.value, 2)
                    if event.axis == 4:
                        arm_xyr.axis[1] = round(event.value, 2)
            except:
                print("No arm joy")
                pass
            #try:
            #    if drive_cont.check_stick(joystick.Joystick(event.joy)):
            #        if event.axis == 0:
           #             drive_cont_xy.axis[0] = round(event.value, 2)
            #        elif event.axis == 1:
            #            drive_cont_xy.axis[1] = round(event.value, 2)
            #        elif event.axis == 3:
            #            drive_cont_throttle.axis[0] = round(event.value, 2)
            #        elif event.axis == 4:
            #            drive_cont_left.axis[0] = -round(event.value, 2)
            #            cont_str = -(round(event.value, 2)+1)/2
            #        elif event.axis == 5:
             #           drive_cont_right.axis[0] = -round(event.value, 2)
            #            cont_str = (round(event.value, 2)+1)*2
            #except:
            #    print("No drive cont")
            #    pass
        # joy hat buttons
        if event.type == py.JOYHATMOTION:
            try:
                if drive_joy.check_stick(joystick.Joystick(event.joy)):
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
                print("no hat")
                pass

            try:
                if arm_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.value[1] == 1:
                        ac_hor = 1
                    if event.value[1] == -1:
                        ac_hor = -1
                    if event.value[0] == 1:
                        ac_vert = 1
                    if event.value[0] == -1:
                        ac_vert = -1
                    if event.value[1] == 0:
                        ac_hor = 0
                    if event.value[0] == 0:
                        ac_vert = 0
            except:
                print("no hat")
                pass

        # Joy buttons
        if event.type == py.JOYBUTTONDOWN:
            try:
                if drive_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.button == 4:
                        steer_option.buttons[0] = 1
                        steer_option.buttons[1] = 0
                        steer_option.buttons[2] = 0
                        steer_option.buttons[3] = 0
                    if event.button == 1:
                        steer_option.buttons[0] = 0
                        steer_option.buttons[1] = 1
                        steer_option.buttons[2] = 0
                        steer_option.buttons[3] = 0
                    if event.button == 5:
                        steer_option.buttons[0] = 0
                        steer_option.buttons[1] = 0
                        steer_option.buttons[2] = 1
                        steer_option.buttons[3] = 0
                    if event.button == 3:
                        steer_option.buttons[0] = 0
                        steer_option.buttons[1] = 0
                        steer_option.buttons[2] = 0
                        steer_option.buttons[3] = 1
                    if event.button == 0:
                        rover.kill_all()

            except:
                print("no drive joy")
                pass
            #try:
             #   if drive_cont.check_stick(joystick.Joystick(event.joy)):
             #       if event.button == 0:
             #           steer_option.buttons[0] = 1
             #           steer_option.buttons[1] = 0
             #           steer_option.buttons[2] = 0
            #        if event.button == 3:
             #           steer_option.buttons[0] = 0
             #           steer_option.buttons[1] = 1
              #          steer_option.buttons[2] = 0
              #      if event.button == 2:
             #           steer_option.buttons[0] = 0
             #           steer_option.buttons[1] = 0
             #           steer_option.buttons[2] = 1
             #       if event.button == 1:
              #          rover.kill_all()
              #      if event.button == 6:
             #           rover.enable_autonomous()
              #          auto_en.buttons[0] = 1
             #           auto_en.buttons[1] = 0
             #       if event.button == 7:
             #           rover.disable_autonomous()
             #           auto_en.buttons[0] = 0
              #          auto_en.buttons[1] = 1

            #except:
            #    print("no drive cont")
            #    pass
            try:
                if arm_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.button == 4:
                        arm_bumper.buttons[1] = 1
                        arm_cl_close = True
                    if event.button == 5:
                        arm_bumper.buttons[0] = 1
                        arm_cl_open = True

            except:
                print('no arm joy')
                pass
        if event.type == py.JOYBUTTONUP:
            try:
                if arm_joy.check_stick(joystick.Joystick(event.joy)):
                    if event.button == 4:
                        arm_bumper.buttons[1] = 0
                        arm_cl_close = False
                    if event.button == 5:
                        arm_bumper.buttons[0] = 0
                        arm_cl_open = False

            except:
                print('no arm joy')

    # set positions for button controlled axis
    if m_drive == -1 and drive_manual_xy.axis[1] <= 1:
        drive_manual_xy.axis[1] += 0.1
    elif m_drive == 1 and drive_manual_xy.axis[1] >= -1:
        drive_manual_xy.axis[1] -= 0.1
    if m_steer == 1 and drive_manual_xy.axis[0] <= 1:
        drive_manual_xy.axis[0] += 0.1
    elif m_steer == -1 and drive_manual_xy.axis[0] >= -1:
        drive_manual_xy.axis[0] -= 0.1

    # set position for manual axis
    if ac_hor == -1 and arm_cam_control.axis[1] <= 1:
        arm_cam_control.axis[1] += 0.05
    elif ac_hor == 1 and arm_cam_control.axis[1] >= -1:
        arm_cam_control.axis[1] -= 0.05
    if ac_vert == 1 and arm_cam_control.axis[0] <= 1:
        arm_cam_control.axis[0] += 0.05
    elif ac_vert == -1 and arm_cam_control.axis[0] >= -1:
        arm_cam_control.axis[0] -= 0.05

    # set position for hook
    if hook_control.buttons[0] == 1 and hook <= 1:
        hook += 0.02
    elif hook_control.buttons[1] == 1 and hook >= -1:
        hook -= 0.02

    # set wheel drive and position
    if joy is True:
        d_lf, d_lm, d_lb, d_rf, d_rm, d_rb, str_lf, str_lb, str_rf, str_rb = calc_drive(steer_option.buttons[0], steer_option.buttons[1], steer_option.buttons[2], steer_option.buttons[3], drive_xy.axis[1], drive_xy.axis[0], drive_xy.axis[1], drive_rot.axis[0], drive_rot.axis[0], manual_control.buttons, drive_manual_xy.axis[1], drive_manual_xy.axis[0], drive_sens.axis[0], -.5)
    else:
        #set_drive(steer_option.buttons[0], steer_option.buttons[1], steer_option.buttons[2], drive_cont_xy.axis[1],
                  #drive_xy.axis[0], drive_cont_throttle.axis[0], cont_str, cont_str, 0)
        pass

    # set wheel velocities and positions
    rover.set_drive(d_lf, d_lm, d_lb, d_rf, d_rm, d_rb, str_lf, str_lb, str_rf, str_rb)

    # set cam
    rover.set_arm_cam(arm_cam_control.axis[0]*100, arm_cam_control.axis[1]*100)

    # set util
    '''
    if util_control.buttons[0] == 1:
        rover.set_util(100)
    elif util_control.buttons[1] == 1:
        rover.set_util(-100)
    else:
        rover.set_util(0)
        '''

    # set hook
    #rover.set_hook(hook*100)

    # set sci
    '''
    if sci_control.buttons[0] == 1:
        rover.set_sci(0)
    elif sci_control.buttons[1] == 1:
        rover.set_sci(-44)
    else:
        pass
        #rover.set_sci(0)
        '''
    '''
    if lumin_control.buttons[0] == 0:
        rover.set_button(95)
    elif lumin_no == 1:
        rover.set_button(-200)
    else:
        rover.set_button(-45)
        '''



    # set calibration
    if steer_option.buttons[3] == 1:
        rover.set_cal(steer_cal.buttons[0], steer_cal.buttons[1], steer_cal.buttons[2], steer_cal.buttons[3])

    if arm_enable.buttons[1] == 1:
        # set arm velocities
        cl_control = \
            set_arm(arm_xyl.axis[0]*(-0.5*arm_sr_scale.axis[0]+0.5), arm_xyl.axis[1]*(-0.5*arm_sp_scale.axis[0]+0.5), ep_vel*(-0.5*arm_ep_scale.axis[0]+0.5),
                    arm_xyr.axis[0]*(-0.5*arm_wr_scale.axis[0]+0.5), arm_xyr.axis[1]*(-0.5*arm_wp_scale.axis[0]+0.5), arm_cl_open,
                    arm_cl_close, cl_control)
    elif arm_enable.buttons[0] == 1:
        cl_control = calc_arm(arm_xyl.axis[1], arm_xyl.axis[0], ep_vel, arm_xyr.axis[0], arm_xyr.axis[1], arm_cl_open,
                              arm_cl_close, cl_control)
        #print('aaaaaaaaaaaaaaaaaaaaa')

    # draw axes in center of screen
    # draw axes for drive joystick
    if joy is True:
        drive_xy.draw(win, infofont, (255, 255, 255), (0, 255, 0))
        drive_sens.draw(win, infofont, (255, 255, 255), (255, 255, 255))
        drive_rot.draw(win, infofont, (255, 255, 255), (0, 0, 255))
    # draw axes for drive controller
    else:
        drive_cont_xy.draw(win, infofont, (255, 255, 255), (0, 255, 0))
        drive_cont_throttle.draw(win, infofont, (255, 255, 255), (255, 255, 255))
        drive_cont_left.draw(win, infofont, (255, 255, 255), (0, 0, 255))
        drive_cont_right.draw(win, infofont, (255, 255, 255), (0, 0, 255))

    arm_xyl.draw(win, infofont, (255, 255, 255), (255, 0, 0))
    arm_xyr.draw(win, infofont, (255, 255, 255), (255, 0, 0))
    arm_trigl.draw(win, infofont, (255, 255, 255), (255, 255, 0))
    arm_trigr.draw(win, infofont, (255, 255, 255), (255, 255, 0))
    arm_bumper.draw(win, infofont, (255, 255, 255), (255, 255, 0))
    #point_steer.draw(win, infofont, (255, 255, 255), (21, 25, 101))
    steer_option.draw(win, infofont, (255, 255, 255), (145, 199, 136))
    manual_control.draw(win, infofont, (255, 255, 255), (21, 25, 101))
    drive_manual_xy.draw(win, infofont, (255, 255, 255), (21, 25, 101))
    arm_cam_control.draw(win, infofont, (255, 255, 255), (240, 89, 69))
    util_control.draw(win, infofont, (255, 255, 255), (240, 89, 69))
    hook_control.draw(win, infofont, (255, 255, 255), (240, 89, 69))
    sci_control.draw(win, infofont, (255, 255, 255), (240, 89, 69))
    lumin_control.draw(win, infofont, (255, 255, 255), (240, 89, 69))
    arm_sr_scale.draw(win, infofont, (255, 255, 255), (255, 0, 0))
    arm_sp_scale.draw(win, infofont, (255, 255, 255), (0, 255, 0))
    arm_ep_scale.draw(win, infofont, (255, 255, 255), (0, 0, 255))
    arm_wr_scale.draw(win, infofont, (255, 255, 255), (255, 255, 0))
    arm_wp_scale.draw(win, infofont, (255, 255, 255), (255, 0, 255))
    arm_scale_control.draw(win, infofont, (255, 255, 255), (128, 0, 0))
    steer_cal.draw(win, infofont, (255, 255, 255), (40, 150, 114))
    #arm_enable.draw(win, infofont, (255, 255, 255), (129, 0, 0))
    auto_en.draw(win, infofont, (255, 255, 255), (105, 48, 195))
    temp_data.draw(win, rover._temperature_getter.value, infofont, (255, 255, 255), (200, 0, 200)) #rover.temperature
    bat1_data.draw(win, rover.voltage[0], infofont, (255, 255, 255), (232, 69, 69)) #rover.voltage[0]
    bat2_data.draw(win, rover.voltage[1], infofont, (255, 255, 255), (232, 69, 69)) #rover.voltage[1]
    long_data.draw(win, rover._gps.longitude, infofont, (255, 255, 255), (91, 138, 114)) #rover._gps.longitude
    lat_data.draw(win, rover._gps.latitude, infofont, (255, 255, 255), (91, 138, 114)) #rover._gps.latitude
    head_data.draw(win, rover._gps.heading, infofont, (255, 255, 255), (91, 138, 114)) #rover._gps.heading
    wp_map.draw(rover._gps.latitude, rover._gps.longitude)
    wp_list.draw()

    """
    gps_map.draw_map(win, northfont, (0, 255, 0), (100, 0, 100))
    if len(gps_map.waypoints_pos) > 0:
        print(gps_map.waypoints_pos[0])
        gps_map.draw_waypoints(win, gpsfont, (0, 0, 0), (255, 0, 0), (255, 255, 255))
    if rover._gps.good_fix is True:
        gps_map.draw_rover(win, rover._gps.longitude, rover._gps.latitude)
    if gps_map.x < mouse_pos[0] < gps_map.x + gps_map.base and gps_map.y < mouse_pos[1] < gps_map.y + gps_map.height:
        gps_map.draw_mouse_pos(win, gpsfont, (255, 255, 255), (0, 0, 0), mouse_pos)
    """
    display.flip()
    #py.time.delay(10)
    win.fill((0, 0, 0))
# end
py.quit()
rover.kill_all()
rover.close()
