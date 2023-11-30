import pygame
import sys
import numpy as np
from time import sleep
from arena import *

# Initialize Pygame
pygame.init()

# Set up display
screen_width, screen_height = 800, 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Path generation test")

# Define arena
ARENA_TYPE="RECTANGLE"
inner_rad=50
robot_pos = np.array([screen_width*.5, screen_height*.5])
obj_pos = robot_pos+np.array([screen_width*.1, screen_height*.1])

if ARENA_TYPE == "RECTANGLE":
    rect_width = 400
    rect_height = 350
    middlePcirc = robot_pos-np.array([200, 0])
    arena = RectangularArena(middleP=robot_pos, width=rect_width, height=rect_height, innerR=inner_rad, middlePCirc=middlePcirc)

elif ARENA_TYPE == "CIRCLE":
    outer_rad = 250
    arena = CircularArena(middleP=robot_pos, innerR=inner_rad, outerR=outer_rad)

# Colors
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
salmon = (255, 128, 0)
magenta = (255, 0, 255)

screen.fill(white)

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    sleep(.2)

    start, end, intersect0, intersect1, succ = arena.generate_waypoints(obj_pos, 5, start_distance=20)

    # Draw stuff    
    if ARENA_TYPE=="CIRCLE":
        pygame.draw.circle(screen, black, (robot_pos[0], robot_pos[1]), outer_rad, 2)
        pygame.draw.circle(screen, red, (obj_pos[0], obj_pos[1]), 7)
        if inner_rad:
            pygame.draw.circle(screen, black, (robot_pos[0], robot_pos[1]), inner_rad, 2)

    elif ARENA_TYPE=="RECTANGLE":
        rect_x = robot_pos[0]-1/2*rect_width
        rect_y = robot_pos[1]-1/2*rect_height #minus nach oben lang in y lul
        outline_rect = pygame.Rect(rect_x, rect_y, rect_width, rect_height)
        pygame.draw.rect(screen, black,  outline_rect, 2)    
        pygame.draw.line(screen, blue, (intersect0[0], intersect0[1]), (intersect1[0], intersect1[1]), 2)
        if inner_rad:
            if len(middlePcirc):
                pygame.draw.circle(screen, black, (middlePcirc[0], middlePcirc[1]), inner_rad, 2)
            else:
                pygame.draw.circle(screen, black, (robot_pos[0], robot_pos[1]), inner_rad, 2)

    if succ:
        pygame.draw.circle(screen, salmon, (start[0], start[1]), 7)
        pygame.draw.circle(screen, magenta, (end[0], end[1]), 7)

        pygame.draw.circle(screen, green, (intersect0[0], intersect0[1]), 7)
        pygame.draw.circle(screen, green, (intersect1[0], intersect1[1]), 7)
        
        pygame.draw.line(screen, blue, (intersect0[0], intersect0[1]), (intersect1[0], intersect1[1]), 2)

    pygame.draw.circle(screen, red, (obj_pos[0], obj_pos[1]), 7)

    # Update the display
    pygame.display.flip()
    
# Quit Pygame
pygame.quit()
sys.exit()
