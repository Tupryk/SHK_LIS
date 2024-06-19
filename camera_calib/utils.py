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


def sample_points(trapezoidal_prism_corners, n):
    # Extracting coordinates of the corners
    x = [corner[0] for corner in trapezoidal_prism_corners]
    y = [corner[1] for corner in trapezoidal_prism_corners]
    z = [corner[2] for corner in trapezoidal_prism_corners]

    # Sorting the points by z-coordinate
    sorted_corners = sorted(trapezoidal_prism_corners, key=lambda p: p[2])

    # Define planes
    bottom_plane = np.array(sorted_corners[:4])
    top_plane = np.array(sorted_corners[4:])
    
    # Sample points within the trapezoidal prism
    sampled_points = []
    while len(sampled_points) < n:
        # Sample random point in the base
        base_point = np.random.uniform(bottom_plane[0], bottom_plane[1])
        
        # Calculate corresponding point in the top plane
        height_ratio = np.random.uniform(0, 1)
        top_point = bottom_plane[0] + height_ratio * (top_plane[0] - bottom_plane[0]) + \
                    (1 - height_ratio) * (bottom_plane[1] - bottom_plane[0])
        
        # Sample a point within the line segment connecting base_point and top_point
        t = np.random.uniform(0, 1)
        sampled_point = base_point + t * (top_point - base_point)
        
        sampled_points.append(tuple(sampled_point))
        
    return sampled_points


from scipy.spatial import Delaunay

def is_point_in_prism(point, vertices):
    """
    Check if a point is inside the trapezoidal prism using Delaunay triangulation.
    
    :param point: 3D point to check
    :param vertices: 8 vertices of the trapezoidal prism
    :return: True if point is inside the prism, False otherwise
    """
    hull = Delaunay(vertices)
    return hull.find_simplex(point) >= 0

def sample_points_from_trapezoid(vertices, n):
    """
    Sample n random points from a trapezoidal prism defined by 8 vertices.
    
    :param vertices: A 2D numpy array of shape (8, 3) containing the coordinates of the 8 vertices
    :param n: Number of random points to sample
    :return: A 2D numpy array of shape (n, 3) containing the sampled points
    """
    # Find the bounding box of the prism
    min_corner = np.min(vertices, axis=0)
    max_corner = np.max(vertices, axis=0)

    sampled_points = []
    while len(sampled_points) < n:
        # Generate random points within the bounding box
        random_points = np.random.uniform(min_corner, max_corner, size=(n, 3))
        
        # Filter points that lie within the trapezoidal prism
        valid_points = [point for point in random_points if is_point_in_prism(point, vertices)]
        
        # Add valid points to the sampled points list
        sampled_points.extend(valid_points)
        
        # Ensure we do not exceed the required number of points
        sampled_points = sampled_points[:n]
        
    return np.array(sampled_points)

