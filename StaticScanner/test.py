import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from helperFunctions import generate_path_on_partial_spherical_surface

def uniform_sample_partial_sphere_surface(num_points, theta_range, radius=1.0, center=(0, 0, 0)):
    """
    Generate uniformly distributed points on a partial spherical surface defined by a range of polar angles.
    
    Args:
        num_points (int): Number of points to generate.
        theta_range (tuple): Range of polar angles (theta) in radians as (theta_min, theta_max).
        radius (float): Radius of the spherical surface.
        center (tuple): Coordinates of the center of the spherical surface.
    
    Returns:
        np.array: Array of points with shape (num_points, 3) representing coordinates on the partial sphere surface.
    """

    phi = np.arange(num_points) / (num_points-1) * np.pi * 2
    theta = theta_range[0] + np.arange(num_points) / num_points * (theta_range[1] - theta_range[0])

    x = center[0] + radius * np.sin(theta) * np.cos(phi)
    y = center[1] + radius * np.sin(theta) * np.sin(phi)
    z = center[2] + radius * np.cos(theta)

    points = np.vstack((x, y, z)).T
    return points

# Generate points on a partial spherical surface (e.g., spherical cap)
num_points = 50
theta_range = (np.radians(0), np.radians(90))
radius = 2.0
center = (0, .2, .5)
points = generate_path_on_partial_spherical_surface(num_points, theta_range, (0,np.pi) , radius, center)

# Plot the points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Uniformly Sampled Points on Partial Sphere Surface')

# Adjust axes limits to show a spherical cap
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3 ])
ax.set_zlim([-3, 3])  # Only showing points above the XY plane

# Set aspect ratio
ax.set_box_aspect([1, 1, 1])

plt.show()
