import numpy as np
import open3d as o3d

def generate_cuboid_meshgrid_border(x_range, y_range, z_range, x_points, y_points, z_points):
    # Generate points along the outer edges
    x_edges = np.linspace(*x_range, x_points)
    y_edges = np.linspace(*y_range, y_points)
    z_edges = np.linspace(*z_range, z_points)

    edge_points = []

    # Generate points along the edges parallel to x-axis
    for y in y_edges:
        for z in z_edges:
            edge_points.append([x_range[0], y, z])
            edge_points.append([x_range[1], y, z])

    # Generate points along the edges parallel to y-axis
    for x in x_edges:
        for z in z_edges:
            edge_points.append([x, y_range[0], z])
            edge_points.append([x, y_range[1], z])

    # Generate points along the edges parallel to z-axis
    for x in x_edges:
        for y in y_edges:
            #edge_points.append([x, y, z_range[0]])
            edge_points.append([x, y, z_range[1]])

    return np.array(edge_points)

# Define ranges and number of points
x_range = (0, 0.12)
y_range = (0, 0.039)
z_range = (0, 0.039)
x_points = 20*round(x_range[-1]/y_range[-1])
y_points = 20
z_points = 20

# Generate meshgrid border points
cuboid_meshgrid_border = generate_cuboid_meshgrid_border(x_range, y_range, z_range, x_points, y_points, z_points)

# Create a PointCloud object for visualization
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(cuboid_meshgrid_border)

# Visualize the cuboid meshgrid border
o3d.visualization.draw_geometries([pcd])

# Save the cuboid meshgrid border
o3d.io.write_point_cloud("data/block/point_cloud_1.pcd", pcd)
