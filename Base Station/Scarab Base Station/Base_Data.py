# install modules
import pygame as py
import pygame.event as event
import pygame.key as key
import pygame.display as display
import pygame.draw as draw
import pygame.mouse as mouse
import pygame.image as image
from geographiclib.geodesic import Geodesic


# data and functions for joysticks
class Joysticks:
    def __init__(self, joy):
        self.joy = joy
        self.id = joy.get_id()
        self.name = joy.get_name()

    # check if stick event came from this stick
    def check_stick(self, stick):
        if stick is None:
            return False
        if self.joy is None:
            return False
        if stick.get_id() == self.id:
            return True
        return False


# data and functions for axises
class Axis:
    # orientation can be 'vertical', 'horizontal'
    def __init__(self, axises, x, y, shape, label):
        self.short = 15
        self.long = 100
        self.square = 10
        self.axis = axises
        self.dimension = len(axises)
        self.x = x
        self.y = y
        self.label = label
        self.rectangle = True
        if shape == "vertical":
            self.vertical = True
        elif shape == "horizontal":
            self.vertical = False
        elif shape == "rectangle":
            self.rectangle = True
        elif shape == "circle":
            self.rectangle = False
        else:
            self.vertical = False
            self.rectangle = False
            print("ERROR: Unknown orientation value, defaulting to horizontal and rectangle")

    # set the position of the stick indicator, length is in pixels
    def set_stick_pos(self, stickvalue, length):
        position = int(stickvalue*length/2)
        return position

    # draw the axises
    def draw(self, screen, font, fontcolor, color):
        # set location and size
        # for 1D
        if self.dimension == 1:
            if self.vertical:
                rect = [self.x-self.short/2, self.y-self.long/2, self.short, self.long]
                dot = [self.x-self.square/2, self.y-self.square/2 + self.set_stick_pos(self.axis[0], self.long-10),
                       self.square, self.square]
                lpos = (self.x, self.y+self.long/2+10)
            else:
                rect = [self.x-self.long/2, self.y-self.short/2, self.long, self.short]
                dot = [self.x-self.square/2 + self.set_stick_pos(self.axis[0], self.long-10), self.y-self.square/2,
                       self.square, self.square]
                lpos = (self.x, self.y+20)
        # for 2D
        elif self.dimension == 2:
            rect = [self.x-self.long/2, self.y-self.long/2, self.long, self.long]
            dot = [self.x-self.square/2 + self.set_stick_pos(self.axis[0], self.long-10),
                   self.y-self.square/2 + self.set_stick_pos(self.axis[1], self.long-10), self.square, self.square]
            lpos = (self.x, self.y + self.long / 2 + 10)
        else:
            print("ERROR: invalid dimension, check axis input")
            return

        # draw
        if not self.rectangle:
            draw.circle(screen, color, [self.x, self.y], int(self.long/2), 5)
            draw.rect(screen, color, dot)
        else:
            draw.rect(screen, color, rect, 5)
            draw.rect(screen, color, dot)

        # label
        label = font.render(self.label, True, fontcolor, None)
        lrect = label.get_rect()
        lrect.center = lpos
        screen.blit(label, lrect)


# data and functions for buttons
class Button:
    def __init__(self, buttons, x, y, shape, label):
        self.buttons = buttons
        self.label = label
        self.count = len(buttons)
        self.x = x
        self.y = y
        self.base = 75
        self.height = 50
        self.shape = 0
        if shape == "cross" and self.count == 4:
            self.shape = 0
        elif shape == "cross" and self.count != 4:
            self.shape = 1
            print("ERROR: Invalid amount of buttons for cross, defaulting to column")
        elif shape == "column":
            self.shape = 1
        elif shape == "row":
            self.shape = 2
        else:
            self.grid = 1
            print("ERROR: Unknown input for shape, defaulting to grid")

    def get_state(self, i):
        if self.buttons[i] == 1:
            return 0
        else:
            return 5

    # draw the buttons
    def draw(self, screen, font, fontcolor, color):
        # set location and size location is based of top button of grid
        # for column
        if self.shape == 1:
            for i in range(0, self.count):
                rect = [self.x-self.base/2, self.y-self.height/2+self.height*i, self.base, self.height]
                label = font.render(self.label[i], True, fontcolor, None)
                lrect = label.get_rect()
                lrect.center = (self.x, self.y+self.height*i)
                draw.rect(screen, color, rect, self.get_state(i))
                screen.blit(label, lrect)
        # for row
        elif self.shape == 2:
            for i in range(0, self.count):
                rect = [self.x-self.base/2+self.base*i, self.y-self.height/2, self.base, self.height]
                label = font.render(self.label[i], True, fontcolor, None)
                lrect = label.get_rect()
                lrect.center = (self.x+self.base*i, self.y)
                draw.rect(screen, color, rect, self.get_state(i))
                screen.blit(label, lrect)
        # for cross
        else:
            top = [self.x-self.height/2, self.y-self.base-self.height/2, self.height, self.base]
            bottom = [self.x-self.height/2, self.y+self.height/2, self.height, self.base]
            left = [self.x-self.base-self.height/2, self.y-self.height/2, self.base, self.height]
            right = [self.x+self.height/2, self.y-self.height/2, self.base, self.height]
            tlabel = font.render(self.label[0], True, fontcolor, None)
            blabel = font.render(self.label[1], True, fontcolor, None)
            llabel = font.render(self.label[2], True, fontcolor, None)
            rlabel = font.render(self.label[3], True, fontcolor, None)
            clabel = font.render(self.label[4], True, fontcolor, None)
            trect = tlabel.get_rect()
            brect = blabel.get_rect()
            lrect = llabel.get_rect()
            rrect = rlabel.get_rect()
            crect = clabel.get_rect()
            trect.center = (self.x, self.y-self.base/2-self.height/2)
            brect.center = (self.x, self.y+self.base/2+self.height/2)
            lrect.center = (self.x-self.base/2-self.height/2, self.y)
            rrect.center = (self.x+self.base/2+self.height/2, self.y)
            crect.center = (self.x, self.y)
            draw.rect(screen, color, top, self.get_state(0))
            draw.rect(screen, color, bottom, self.get_state(1))
            draw.rect(screen, color, left, self.get_state(2))
            draw.rect(screen, color, right, self.get_state(3))
            screen.blit(tlabel, trect)
            screen.blit(blabel, brect)
            screen.blit(llabel, lrect)
            screen.blit(rlabel, rrect)
            screen.blit(clabel, crect)


class Data:
    def __init__(self, title, x, y, base, units, figs):
        self.theight = 25
        self.dheight = 25
        self.base = base
        self.x = x
        self.y = y
        self.title = title
        self.units = units
        self.figs = figs

    def draw(self, screen, value, font, fontcolor, color):
        tlabel = font.render(self.title, True, fontcolor, None)
        if value is None:
            dlabel = font.render("N/A", True, fontcolor, None)
        else:
            x = float(value)
            dlabel = font.render("{0:.{r}f} {1}".format(x, self.units, r=self.figs), True, fontcolor, None)
        trect = tlabel.get_rect()
        trect.center = (self.x, self.y-self.theight/2)
        drect = dlabel.get_rect()
        drect.center = (self.x, self.y+self.dheight/2)
        screen.blit(tlabel, trect)
        screen.blit(dlabel, drect)
        draw.rect(screen, color, (self.x-self.base/2, self.y-self.theight, self.base, self.theight), 2)
        draw.rect(screen, color, (self.x-self.base/2, self.y, self.base, self.dheight), 2)


class Map:
    def __init__(self, x, y, map_image, cord):
        self.height = 400
        self.base = 400
        self.x = x-self.base/2
        self.y = y-self.height/2
        self.coord = cord
        self.delta = [self.coord[2] - self.coord[0], self.coord[3] - self.coord[1]]
        self.center_coord = [self.coord[0] + self.delta[0] / 2, self.coord[1] + self.delta[1] / 2]
        self.scale_coord = [0, 0, 0, 0]
        self.icon = py.image.load('rover_icon.png')
        self.waypoints = []
        self.waypoints_pos = []

        if map_image is not None:
            self.map = image.load(map_image)
            size = self.map.get_size()
            self.b = size[0]
            self.h = size[1]
            self.base = int(size[0]*0.35)
            self.height = int(size[1]*0.35)
            self.scale_map = py.transform.smoothscale(self.map, (self.base, self.height))
            self.scale = 1
            self.map_pos = [self.base/(0.35*2), self.height/(0.35/2)]

    def draw_map(self, screen, font, fontcolor, color):
        if self.scale_map is not None:
            rect = py.Rect(self.x, self.y, self.base, self.height)
            screen.blit(self.scale_map, rect)
        label = font.render('N', True, fontcolor, None)
        rect = label.get_rect()
        rect.center = (self.x+self.base/2, self.y-10)
        screen.blit(label, rect)
        label = font.render("{0:.4f} {1:.4f}".format(self.scale_coord[0], self.scale_coord[1]), True, fontcolor, None)
        rect = label.get_rect()
        rect.center = (self.x+self.base*0.1, self.y-10)
        screen.blit(label, rect)
        label = font.render("{0:.4f} {1:.4f}".format(self.scale_coord[2], self.scale_coord[1]), True, fontcolor, None)
        rect = label.get_rect()
        rect.center = (self.x+self.base*0.9, self.y-10)
        screen.blit(label, rect)
        label = font.render("{0:.4f} {1:.4f}".format(self.scale_coord[0], self.scale_coord[3]), True, fontcolor, None)
        rect = label.get_rect()
        rect.center = (self.x+self.base*0.1, self.y+self.height+10)
        screen.blit(label, rect)
        label = font.render("{0:.4f} {1:.4f}".format(self.scale_coord[2], self.scale_coord[3]), True, fontcolor, None)
        rect = label.get_rect()
        rect.center = (self.x+self.base*0.9, self.y+self.height+10)
        screen.blit(label, rect)
        label = font.render("{0:.4f} {1:.4f}".format(self.center_coord[0], self.center_coord[1]), True, fontcolor, None)
        rect = label.get_rect()
        rect.center = (self.x+self.base/2, self.y+self.height+10)
        screen.blit(label, rect)
        draw.rect(screen, color, (self.x, self.y, self.base, self.height), 5)

    def draw_waypoints(self, screen, font, fontcolor, pcolor, lcolor):
        x = self.map_pos[0]-self.b*self.scale/2
        y = self.map_pos[1]-self.h*self.scale/2
        wp_x_last = 0
        wp_y_last = 0
        for i in range(0, len(self.waypoints_pos)):
            if x < self.waypoints_pos[i][0] < x+self.b*self.scale and y < self.waypoints_pos[i][1] < y+self.h*self.scale:
                wp_x = self.x+self.base*(self.waypoints_pos[i][0]-x)/(self.b*self.scale)
                wp_y = self.y+self.height*(self.waypoints_pos[i][1]-y)/(self.h*self.scale)
                label = font.render(str(i), True, fontcolor, None)
                rect = label.get_rect()
                rect.center = (wp_x, wp_y)
                draw.circle(screen, pcolor, rect.center, 7)
                screen.blit(label, rect)
                if i != 0:
                    if x < self.waypoints_pos[i-1][0] < x + self.b * self.scale and y < self.waypoints_pos[i-1][
                     1] < y + self.h * self.scale:
                        draw.line(screen, lcolor, (wp_x_last, wp_y_last),
                                  (wp_x, wp_y), 2)
                draw.circle(screen, pcolor, rect.center, 7)
                screen.blit(label, rect)
                wp_x_last = wp_x
                wp_y_last = wp_y

    def change_map(self, click_pos, direction):
        self.scale += direction*0.05
        if self.scale < 0.1:
            self.scale = 0.1
        if self.scale > 1:
            self.scale = 1
        pos = [(click_pos[0]-self.x)/0.35, (click_pos[1]-self.y)/0.35]
        self.b = int(self.base/0.35)
        self.h = int(self.height/0.35)
        if direction == 0:
            x = self.map_pos[0] + self.scale * (pos[0] - self.b / 2) - self.b * self.scale / 2
            y = self.map_pos[1] + self.scale * (pos[1] - self.h / 2) - self.h * self.scale / 2
        else:
            x = self.map_pos[0]+self.scale*(pos[0]-self.b/2) - self.b * self.scale / 2
            y = self.map_pos[1]+self.scale*(pos[1]-self.h/2) - self.h * self.scale / 2
        if x < 0:
            x = 0
        elif x + self.b*self.scale > self.b:
            x = self.b * (1 - self.scale)
        if y < 0:
            y = 0
        elif y + self.h*self.scale > self.h:
            y = self.h * (1 - self.scale)
        self.scale_map = self.map.subsurface((x, y, self.b*self.scale, self.h*self.scale))
        self.scale_map = py.transform.smoothscale(self.scale_map, (self.base, self.height))
        self.map_pos = [x+self.b*self.scale/2, y+self.h*self.scale/2]
        self.center_coord = [0, 0]
        self.center_coord[0] = self.coord[0]+self.delta[0]*self.map_pos[0]/self.b
        self.center_coord[1] = self.coord[1]+self.delta[1]*self.map_pos[1]/self.h
        self.scale_coord[0] = self.center_coord[0]-self.delta[0]*(self.b*self.scale/2)/self.b
        self.scale_coord[1] = self.center_coord[1]-self.delta[1]*(self.h*self.scale/2)/self.h
        self.scale_coord[2] = self.center_coord[0]+self.delta[0]*(self.b*self.scale/2)/self.b
        self.scale_coord[3] = self.center_coord[1]+self.delta[1]*(self.h*self.scale/2)/self.h

    def draw_mouse_pos(self, screen, font, fontcolor, color, pos):
        lon = self.scale_coord[0]+((pos[0]-self.x)/self.base)*self.delta[0]*self.scale
        lat = self.scale_coord[1]+((pos[1]-self.y)/self.height)*self.delta[1]*self.scale
        label = font.render("{0:.4f} {1:.4f}".format(lon, lat), True, fontcolor, None)
        rect = label.get_rect()
        rect.center = (pos[0], pos[1]-5)
        size = label.get_size()
        draw.rect(screen, color, (pos[0]-size[0]/2, pos[1]-size[1]/2-5, size[0], size[1]), 0)
        screen.blit(label, rect)

    def draw_rover(self, screen, lon, lat):
        if self.scale_coord[0] <= lon <= self.scale_coord[2] and self.scale_coord[1] <= lat <= self.scale_coord[3]:
            x = int(((lon-self.scale_coord[0])/(self.delta[0]*self.scale))*self.base)
            y = int(((lat-self.scale_coord[1])/(self.delta[1]*self.scale))*self.height)
            pos = self.icon.get_rect()
            pos.center = (x, y)
            screen.blit(self.icon, pos)

    def add_waypoint(self, rover, pos):
        x = self.map_pos[0]+((pos[0]-(self.x+self.base/2))/self.base)*self.b*self.scale
        y = self.map_pos[1]+((pos[1]-(self.y+self.height/2))/self.height)*self.h*self.scale
        lon = self.coord[0]+self.delta[0]*x/self.b
        lat = self.coord[1]+self.delta[1]*y/self.h
        self.waypoints_pos.append([int(x), int(y)])
        self.waypoints.append([lon, lat])
        rover.add_waypoint(lon, lat)

    def forget_waypoints(self, rover):
        self.waypoints_pos = []
        self.waypoints = []
        rover.forget_waypoints()

