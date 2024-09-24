import random
import math
import robotic as ry
 
def sample_elliptical_arena(a = .4, b = .4, z_coord = .745):
    a = 0.4
    b = 0.35  
    z_coord = 0.745  # Fixed Z-coordinate
 
    r = math.sqrt(random.uniform(0, 1))
 
    angle = random.uniform(0, 2 * math.pi)
 
    x = r * a * math.cos(angle)
    y = 0.2 + r * b * math.sin(angle)  # Centered at (0, 0.2)
 
    return [x, y, z_coord]
 
 
def draw_arena(C : ry.Config, a = .4, b = .4, z_coord = .745):
    for i in range(100):
        angle = random.uniform(0, 2 * math.pi)
 
        x = a * math.cos(angle)
        y = b * math.sin(angle)
        midpoint = [x, y, z_coord]
 
        C.addFrame(f"point{i}").setPosition(midpoint).setShape(ry.ST.sphere, size=[.01]).setColor([.5, .5, .5])