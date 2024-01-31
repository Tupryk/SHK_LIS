import pygame
import numpy as np


pygame.init()
WIDTH, HEIGT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGT))
pygame.display.set_caption("View Angle Visualization")


push_mean = np.array([500, 300])
robot_pos = np.array([200, 300])
robot_radius = 75
max_gripper_distance = 400
min_gripper_distance = 200
max_push_mean_distance = 200

def inArena(point: np.ndarray):
    return point[0] >= 100 and point[0] < 100+600 and point[1] >= 100 and point[1] < 100+400

def probToColor(prob: float):
    r = 255 * prob
    return (r, 0, 0)

def drawVector(position: np.ndarray, direction: np.ndarray, probability: float):
    direction *= 5
    start = position - direction
    end = position + direction

    color = probToColor(probability)
    pygame.draw.line(screen, color, start, end, 2)

    # Pointy bit
    tmp = -direction[0]
    direction[0] =  direction[1]
    direction[1] = tmp
    p1 = position + direction
    p2 = position - direction
    pygame.draw.line(screen, color, end, p1, 2)
    pygame.draw.line(screen, color, end, p2, 2)

def drawStuff():
    ### Push arena (possible object position)
    pygame.draw.rect(screen, (200, 200, 200), pygame.Rect(100, 100, 600, 400))
    pygame.draw.circle(screen, (200, 0, 0), robot_pos, robot_radius)

    ### Look direction vector field
    for i in range(0, WIDTH, 15):
        for j in range(0, HEIGT, 15):

            obj_pos = np.array([float(i), float(j)])
            rob2vec = np.linalg.norm(robot_pos - obj_pos)

            vec = push_mean - obj_pos
            vec_len =  np.linalg.norm(vec)
            vec /= vec_len
            
            if rob2vec <= max_gripper_distance and rob2vec >= min_gripper_distance and inArena(obj_pos) and vec_len <= max_push_mean_distance:
                prob = vec_len / 100
                prob = 1 if prob > 1 else prob
                drawVector(obj_pos, vec, prob)
            

def main():
    screen.fill((125, 125, 125))

    drawStuff()
        
    pygame.display.update()
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

main()
