# install modules
import pygame as py
import pygame.event as event
import pygame.key as key
import pygame.display as display
import pygame.draw as draw
import pygame.mouse as mouse
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
    def set_stick_pos(self, stickValue, length):
        position = int(stickValue*length/2)
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
    def __init__(self, title, x, y, units):
        self.theight = 25
        self.dheight = 25
        self.base = 100
        self.x = x
        self.y = y
        self.title = title
        self.units = units

    def draw(self, screen, value, font, fontcolor, color):
        tlabel = font.render(self.title, True, fontcolor, None)
        dlabel = font.render("%0.2f %s" % (value, self.units), True, fontcolor, None)
        trect = tlabel.get_rect()
        trect.center = (self.x, self.y-self.theight/2)
        drect = dlabel.get_rect()
        drect.center = (self.x, self.y+self.dheight/2)
        screen.blit(tlabel, trect)
        screen.blit(dlabel, drect)
        draw.rect(screen, color, (self.x-self.base/2, self.y-self.theight, self.base, self.theight), 2)
        draw.rect(screen, color, (self.x-self.base/2, self.y, self.base, self.dheight), 2)

class Map:
    def __init__(self, x, y):
        self.height = 400
        self.base = 400
        self.x = x-self.base/2
        self.y = y-self.height/2

    def draw_map(self, screen, font, fontcolor, color):
        label = font.render('N', True, fontcolor, None)
        rect = label.get_rect()
        rect.center = (self.x+self.base/2, self.y-10)
        screen.blit(label, rect)
        draw.rect(screen, color, (self.x, self.y, self.base, self.height), 5)

    def draw_rover(self, screen, x, y):
        icon = py.image.load('rover_icon.png')
        pos = icon.get_rect()
        pos.center = (x, y)
        screen.blit(icon, pos)
