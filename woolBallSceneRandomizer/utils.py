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


def sample_rectangular_arena(width=0.4, height=0.4, z_coord=0.745, center_point=[0, 0]):
    # Sample x and y uniformly within the rectangle's bounds
    x = center_point[0] + random.uniform(-width / 2, width / 2)
    y = center_point[1] + random.uniform(-height / 2, height / 2)
    
    return [x, y, z_coord]

def draw_rectangular_arena(C, width=0.4, height=0.4, z_coord=0.745, center_point=[0, 0]):
    # Define the four corners of the rectangle
    corners = [
        [center_point[0] - width / 2, center_point[1] - height / 2, z_coord],  # Bottom-left
        [center_point[0] + width / 2, center_point[1] - height / 2, z_coord],  # Bottom-right
        [center_point[0] + width / 2, center_point[1] + height / 2, z_coord],  # Top-right
        [center_point[0] - width / 2, center_point[1] + height / 2, z_coord]   # Top-left
    ]
    
    # Number of points to draw on each side of the rectangle
    points_per_side = 25
    total_points = 4 * points_per_side

    for i in range(total_points):
        # Determine which side of the rectangle we are on
        side = i // points_per_side
        t = (i % points_per_side) / points_per_side
        
        # Linearly interpolate between two consecutive corners
        if side == 0:
            # Bottom side (between bottom-left and bottom-right)
            x = (1 - t) * corners[0][0] + t * corners[1][0]
            y = (1 - t) * corners[0][1] + t * corners[1][1]
        elif side == 1:
            # Right side (between bottom-right and top-right)
            x = (1 - t) * corners[1][0] + t * corners[2][0]
            y = (1 - t) * corners[1][1] + t * corners[2][1]
        elif side == 2:
            # Top side (between top-right and top-left)
            x = (1 - t) * corners[2][0] + t * corners[3][0]
            y = (1 - t) * corners[2][1] + t * corners[3][1]
        elif side == 3:
            # Left side (between top-left and bottom-left)
            x = (1 - t) * corners[3][0] + t * corners[0][0]
            y = (1 - t) * corners[3][1] + t * corners[0][1]
        
        midpoint = [x, y, z_coord]
        C.addFrame(f"point{i}").setPosition(midpoint).setShape(ry.ST.marker, size=[.06]).setColor([.5, .5, .5])

