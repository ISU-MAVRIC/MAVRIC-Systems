import pygame
from math import sin, cos, pi, sqrt

#All you really need to edit is getting numbers into A

A = 300
#Length of the first link (from the shoulder to elbow)
a = pi/4
# angles are in radians
originX = 10
originY = 590
#where on the screen the lines should start
running = 1
#helper variable for the program to start and stop it properly
width = 800
height = 600
#Width and height of the window on startup
screen = pygame.display.set_mode((width, height))
#Sets screen to the width and height of the window
linecolor1 = 255, 0, 0
#Color of the arm
linecolor2 = 255, 255, 0
#Color of the axes
bgcolor = 0, 0, 0
B = A * cos(a)
C = A * sin(a)

while running:
    event = pygame.event.poll()
    if event.type == pygame.QUIT:
        running = 0

    screen.fill(bgcolor)
    pygame.draw.lines(screen, linecolor1, False,  ((originX,originY),(originX + B, originY - C),(originX + B + 100, originY - C), (originX + B + 200, originY - 100), 1))
	#So I drew the lines in a rough arm shape for visualization, you're going to want to find a way to get the data into points and draw the lines with the proof of concept.
    pygame.draw.line(screen, linecolor2, (10,10), (10,height - 10))
    pygame.draw.line(screen, linecolor2, (10,height - 10), (width - 10,height - 10))
	#Draws axes
    B+=.01;
    C+=.01;
	#Simple animation proof of concept, you're going to want to update your variables here

    pygame.display.flip()
	
def aaas(D, E, F, f):
	#Helper function in case you might need it
    """ This function solves the triangle and returns (d,e,f,D,E,F) """
    d = f * sin(D) / sin(F)
    e = f * sin(E) / sin(F)
    return (d,e,f,D,E,F)