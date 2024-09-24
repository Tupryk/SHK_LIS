import random
import math
import robotic as ry
import numpy as np

def sample_elliptical_arena(a = .4, b = .4, z_coord = .745, center_point = [0,0]): 
    r = math.sqrt(random.uniform(0, 1))
 
    angle = random.uniform(0, 2 * math.pi)
 
    x = center_point[0] + r * a * math.cos(angle)
    y = center_point[1] + r * b * math.sin(angle)  
 
    return [x, y, z_coord]
 
 
def draw_elliptical_arena(C : ry.Config, a = .4, b = .4, z_coord = .745, center_point = [0,0]):
    for i in range(100):
        angle = random.uniform(0, 2 * math.pi)
 
        x = center_point[0] + a * math.cos(angle)
        y = center_point[1] + b * math.sin(angle)
        midpoint = [x, y, z_coord]
 
        C.addFrame(f"point{i}").setPosition(midpoint).setShape(ry.ST.marker, size=[.06]).setColor([.5, .5, .5])
