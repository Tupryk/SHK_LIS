import numpy as np
import open3d as o3d
import open3d.visualization as vis


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


# Define a custom gripper shape composed of cuboids
def create_gripper_geometry():
    gripper = o3d.geometry.TriangleMesh()

    # Create individual cuboids for the gripper
    # You can adjust dimensions and positions to form the desired gripper shape
    cuboid1 = o3d.geometry.TriangleMesh.create_box(width=0.02, height=0.02, depth=0.05)
    cuboid2 = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.03, depth=0.05)
    cuboid3 = o3d.geometry.TriangleMesh.create_box(width=0.02, height=0.02, depth=0.05)

    # Translate and assemble cuboids to form the gripper shape
    cuboid1.translate([-.005-.04, .005, -.05])
    cuboid2.translate([-.045, .0, .0])
    cuboid3.translate([-.005+.04, .005, -.05])
    # Combine cuboids into a single mesh (gripper)
    gripper += cuboid2
    gripper += cuboid1
    gripper += cuboid3
    # Compute vertex normals for visualization
    gripper.compute_vertex_normals()

    return gripper
# Step 1: Load the point cloud from a .pcd file
pcd_path = "registrated.pcd"
point_cloud = o3d.io.read_point_cloud(pcd_path)


sphere_radius = 0.005  # Radius of the sphere used 

# Step 2: Estimate normals for the point cloud
# Set parameters for normal estimation
radius_normal = 0.1  # radius to search for neighbors for normal estimation

# Estimate normals
point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
#point_cloud.orient_normals_consistent_tangent_plane(k=30)

# Step 3: Visualize the point cloud with normals
# Create visualization window
# o3d.visualization.draw_geometries([point_cloud], window_name="Point Cloud with Normals",
#                                    width=800, height=600, point_show_normal=True)

# Step 4: Randomly sample 100 points and highlight them
num_samples = 1

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
# o3d.visualization.draw_geometries([point_cloud] + highlighted_geometries,
#                                    window_name="Point Cloud with Highlighted Points",
#                                    width=800, height=600)


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
# o3d.visualization.draw_geometries([inside_points],
#                                    window_name="Point Cloud with Reference Frame",
#                                    width=800, height=600)

frame_list= []

for i in range(len(mat_list)):
    eigenvalues, eigenvectors = np.linalg.eig(mat_list[i])

    # Sort eigenvalues and corresponding eigenvectors in ascending order
    sorted_indices = np.argsort(eigenvalues)
    sorted_eigenvalues = eigenvalues[sorted_indices]
    sorted_eigenvectors = eigenvectors[:, sorted_indices]

    # Define custom basis vectors for a right-handed coordinate system
    # Ensure that u, v, w represent a right-handed coordinate system
    # Based on the sorted eigenvectors (column-wise in sorted_eigenvectors)

    # Custom X-axis (u) should be the first eigenvector (corresponding to smallest eigenvalue)
    u = sorted_eigenvectors[:, 0]

    # Custom Y-axis (v) should be the second eigenvector (corresponding to middle eigenvalue)
    v = sorted_eigenvectors[:, 1]

    # Ensure that the cross product of u and v gives a vector in the direction of the third eigenvector (w)
    # Custom Z-axis (w) is the cross product of u and v, normalized to ensure unit length
    w = np.cross(u, v)
    w = w / np.linalg.norm(w)  

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
# o3d.visualization.draw_geometries([point_cloud]+ frame_list,
#                                    window_name="Custom Coordinate Frame with Colored Axes",
#                                    width=800, height=600)

def visualize_grasps(point_cloud, grippers, point):
    # Define materials for sphere, box, and point cloud
    mat_sphere = vis.rendering.MaterialRecord()
    mat_sphere.shader = 'defaultLit'
    mat_sphere.base_color = [0, 0, .8, 1.0]

    mat_box = vis.rendering.MaterialRecord()
    mat_box.shader = 'defaultLitSSR'
    mat_box.base_color = [0.467, 0.467, 0.467, 0.2]
    mat_box.base_roughness = 0.0
    mat_box.base_reflectance = 0.0
    mat_box.base_clearcoat = 1.0
    mat_box.thickness = 1.0
    mat_box.transmission = .5
    mat_box.absorption_distance = 10
    mat_box.absorption_color = [0.5, 0.5, 0.5]

    mat_pcd = vis.rendering.MaterialRecord()
    mat_pcd.shader = 'defaultLit'
    mat_pcd.base_color = [0, 0.8, 0, 1.0]  # Green color for point cloud

    geoms = [{'name': 'point_cloud', 'geometry': point_cloud, 'material': mat_pcd}]
    sphere = o3d.geometry.TriangleMesh.create_sphere(.003)
    sphere.compute_vertex_normals()
    sphere.translate(point)
    geoms.append({'name': 'sphere_-1', 'geometry': sphere, 'material': mat_sphere})

    mat_sphere0 = vis.rendering.MaterialRecord()
    mat_sphere0.shader = 'defaultLit'
    mat_sphere0.base_color = [.8, 0, 0, 1.0]
    for i, gripper in enumerate(grippers):
        sphere = o3d.geometry.TriangleMesh.create_sphere(.003)
        sphere.compute_vertex_normals()
        sphere.translate(gripper.get_center())
        geoms.append({'name': f'gripper_{i}', 'geometry': gripper, 'material': mat_box})
        geoms.append({'name': f'sphere_{i}', 'geometry': sphere, 'material': mat_sphere0})

    vis.draw(geoms)

gripper_geometries = []
for i, point in enumerate(sampled_points):
    gripper = create_gripper_geometry()

    eigenvalues, eigenvectors = np.linalg.eig(mat_list[i])

    # Sort eigenvalues and corresponding eigenvectors in ascending order
    sorted_indices = np.argsort(eigenvalues)
    sorted_eigenvalues = eigenvalues[sorted_indices]
    sorted_eigenvectors = eigenvectors[:, sorted_indices]

    gripper.translate(sorted_eigenvectors[0] * .003, relative=True)

    #gripper.rotate(np.array([j for j in sorted_eigenvectors]))

    # Add the gripper to the list of highlighted geometries
    gripper_geometries.append(gripper)

visualize_grasps(point_cloud, gripper_geometries, point)
# Visualize the updated scene with highlighted grippers
# o3d.visualization.draw_geometries([point_cloud] + frame_list + gripper_geometries, window_name="Point Cloud with Highlighted Grippers", width=800, height=600)

# for i, gg in enumerate(gripper_geometries):
#     ray = o3d.geometry.TriangleRayIntersector()
#     ray.intersect(gg)
#     is_valid = True
#     for point in point_cloud.points:
#         if False: #ray.has_intersection(point):
#             is_valid = False
#             break
#     if is_valid:
#         gripper_geometries[i].paint_uniform_color([0, 1, 0])
#     else:
#         gripper_geometries[i].paint_uniform_color([1, 0, 0])
