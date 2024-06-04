import numpy as np
import random

def sample3dpoints(nPoints, xmin, xmax, ymin, ymax, zmin, zmax):
    points = []
    for _ in range(nPoints):
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        z = random.uniform(zmin, zmax)
        points.append((x, y, z))
    return np.asarray(points)