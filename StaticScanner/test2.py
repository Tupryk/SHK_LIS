import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def uniform_sample_partial_sphere_surface(num_points, theta_range):
    """
    Generate uniformly distributed points on a partial spherical surface defined by a range of polar angles.
    
    Args:
        num_points (int): Number of points to generate.
        theta_range (tuple): Range of polar angles (theta) in radians as (theta_min, theta_max).
    
    Returns:
        np.array: Array of points with shape (num_points, 3) representing coordinates on the partial sphere surface.
    """
    phi = np.random.uniform(0, np.pi * 2, num_points)
    theta = np.random.uniform(theta_range[0], theta_range[1], num_points)

    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)

    points = np.vstack((x, y, z)).T
    return points

def generate_equidistant_points_on_spherical_cap(num_points_theta, num_points_phi, theta_range, phi_range):
    """
    Generate equidistant points on a spherical cap defined by ranges of polar and azimuthal angles.
    
    Args:
        num_points_theta (int): Number of points to generate along the polar angle (theta).
        num_points_phi (int): Number of points to generate along the azimuthal angle (phi).
        theta_range (tuple): Range of polar angles (theta) in radians as (theta_min, theta_max).
        phi_range (tuple): Range of azimuthal angles (phi) in radians as (phi_min, phi_max).
    
    Returns:
        np.array: Array of points with shape (num_points_theta * num_points_phi, 3) representing coordinates on the spherical cap.
    """
    theta_values = np.linspace(theta_range[0], theta_range[1], num_points_theta)
    phi_values = np.linspace(phi_range[0], phi_range[1], num_points_phi)

    points = []
    for theta in theta_values:
        for phi in phi_values:
            x = np.sin(theta) * np.cos(phi)
            y = np.sin(theta) * np.sin(phi)
            z = np.cos(theta)
            points.append([x, y, z])

    return np.array(points)

def generate_evenly_spaced_points_on_spherical_cap(num_points, theta_range, phi_range):
    """
    Generate equidistant points on a spherical cap defined by ranges of polar and azimuthal angles.
    
    Args:
        num_points (int): Number of points to generate.
        theta_range (tuple): Range of polar angles (theta) in radians as (theta_min, theta_max).
        phi_range (tuple): Range of azimuthal angles (phi) in radians as (phi_min, phi_max).
    
    Returns:
        np.array: Array of points with shape (num_points, 3) representing coordinates on the spherical cap.
    """
    theta_values = np.linspace(theta_range[0], theta_range[1], int(np.sqrt(num_points)))
    phi_values = np.linspace(phi_range[0], phi_range[1], int(np.sqrt(num_points)))

    points = []
    for theta in theta_values:
        for phi in phi_values:
            x = np.sin(theta) * np.cos(phi)
            y = np.sin(theta) * np.sin(phi)
            z = np.cos(theta)
            points.append([x, y, z])

    return np.array(points)[:num_points]


# Generate points on a partial spherical surface (e.g., spherical cap)
num_points = 500
theta_range = (np.radians(20), np.radians(60))  # Example: polar angle range from 0 to pi

points = generate_equidistant_points_on_spherical_cap(10, 50, theta_range, (0,2*np.pi))

# Plot the points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Uniformly Sampled Points on Partial Sphere Surface')

# Adjust axes limits to show a spherical cap
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])  # Only showing points above the XY plane

# Set aspect ratio
ax.set_box_aspect([1, 1, 1])

plt.show()
