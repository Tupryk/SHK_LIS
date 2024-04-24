import open3d as o3d
import numpy as np

# Step 1: Load the point cloud from a .pcd file
pcd_path = "registrated.pcd"
point_cloud = o3d.io.read_point_cloud(pcd_path)


sphere_radius = 0.005  # Radius of the sphere used 

# Step 2: Estimate normals for the point cloud
# Set parameters for normal estimation
radius_normal = 0.1  # radius to search for neighbors for normal estimation

# Estimate normals
point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

# Step 3: Visualize the point cloud with normals
# Create visualization window
o3d.visualization.draw_geometries([point_cloud], window_name="Point Cloud with Normals",
                                   width=800, height=600)

# Step 4: Randomly sample 100 points and highlight them
num_samples = 10

# Get the points from the point cloud
points = np.asarray(point_cloud.points)
normals = np.asarray(point_cloud.normals)  # Array of normal vectors (n x 3)

# Randomly sample points from the point cloud
sample_indices = np.random.choice(len(points), size=num_samples, replace=False)
sampled_points = points[sample_indices]

# Create spheres at the sampled points to highlight them
highlighted_geometries = []
for point in sampled_points:
    # Create a sphere geometry at the sampled point
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)  # Adjust radius as needed
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color([1, 0, 0])  # Set color (e.g., red) for the sphere

    # Translate the sphere to the sampled point position
    sphere.translate(point)

    # Add the sphere to the list of highlighted geometries
    highlighted_geometries.append(sphere)

# Visualize the updated scene with highlighted points
o3d.visualization.draw_geometries([point_cloud] + highlighted_geometries,
                                   window_name="Point Cloud with Highlighted Points",
                                   width=800, height=600)

# Step 5: Filter the point cloud to show only points inside the spheres
inside_sphere_indices = []

mat_list = []
point_list = []

for sampled_point in sampled_points:
    # Initialize matrix to accumulate outer products
    sum_outer_product = np.zeros((3, 3))

    # Iterate over each point in the original point cloud
    for idx, pt in enumerate(points):
        # Check if the point is inside the current sampled sphere
        if np.linalg.norm(pt - sampled_point) <= sphere_radius:
            # Accumulate the outer product of normals[idx]
            sum_outer_product += np.outer(normals[idx], normals[idx])
            inside_sphere_indices.append(idx)


    # Append the accumulated outer product to mat_list
    mat_list.append(sum_outer_product)

# Display the accumulated outer products (matrices) for each sampled sphere
for i, mat in enumerate(mat_list):
    print(f"Matrix for sampled point {i}:")
    print(mat)


inside_points = point_cloud.select_by_index(inside_sphere_indices)

# # Visualize the point cloud with only the points inside the spheres
# o3d.visualization.draw_geometries([inside_points],
#                                    window_name="Point Cloud Inside Spheres",
#                                    width=800, height=600)




# Step 3: Visualize the point cloud with the reference frame
# Create visualization window and add geometries
# Define the origin and axis directions for the coordinate frame


# Step 3: Visualize the point cloud with the reference frame
# Create visualization window and add geometries
o3d.visualization.draw_geometries([inside_points],
                                   window_name="Point Cloud with Reference Frame",
                                   width=800, height=600)

frame_list= []

for i in range(len(mat_list)):
    eigenvalues, eigenvectors = np.linalg.eig(mat_list[i])

    # Step 2: Sort eigenvalues and corresponding eigenvectors
    # Get the indices that would sort the eigenvalues in ascending order
    sorted_indices = np.argsort(eigenvalues)
    sorted_eigenvalues = eigenvalues[sorted_indices]
    sorted_eigenvectors = eigenvectors[:, sorted_indices]


    # Step 1: Define custom basis vectors
    u = sorted_eigenvectors[0]  # Custom X-axis
    v = sorted_eigenvectors[1] # Custom Y-axis
    w = sorted_eigenvectors[2]  # Custom Z-axis

    # Step 2: Define scaling factor for the coordinate frame
    scale_factor = 0.07  # Adjust the scale factor as needed

    # Step 3: Create custom coordinate frame as a LineSet geometry
    # Define vertices of the custom coordinate frame and apply scaling
    origin = [sampled_points[i][0], sampled_points[i][1], sampled_points[i][2]]

    vertices = np.array([
        origin,    # Origin
        u * scale_factor + origin,   # Endpoint of custom X-axis (scaled)
        v * scale_factor + origin ,   # Endpoint of custom Y-axis (scaled)
        w * scale_factor + origin    # Endpoint of custom Z-axis (scaled)
    ])

    # Define lines connecting vertices to represent axes
    lines = [[0, 1], [0, 2], [0, 3]]

    # Create LineSet geometry for the custom coordinate frame
    frame_mesh = o3d.geometry.LineSet()
    frame_mesh.points = o3d.utility.Vector3dVector(vertices)
    frame_mesh.lines = o3d.utility.Vector2iVector(lines)

    # Step 4: Assign colors to the axes in the LineSet geometry
    # Define RGB colors for X, Y, Z axes (red, green, blue)
    colors = [[1.0, 0.0, 0.0],  # Red (X-axis)
            [0.0, 1.0, 0.0],  # Green (Y-axis)
            [0.0, 0.0, 1.0]]  # Blue (Z-axis)

    # Set colors for each line (axis) in the LineSet
    line_colors = [colors[i] for i in range(len(lines))]
    frame_mesh.colors = o3d.utility.Vector3dVector(line_colors)
    frame_list.append(frame_mesh)

# Step 5: Visualize the custom coordinate frame with colored axes and scaling
# Create visualization window and add custom coordinate frame
o3d.visualization.draw_geometries([point_cloud]+ frame_list,
                                   window_name="Custom Coordinate Frame with Colored Axes",
                                   width=800, height=600)