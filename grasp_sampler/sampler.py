import numpy as np
import open3d as o3d


def sample_grasps(points, normals, point_cloud, num_samples=1, verbose=0):
    """
    This function gives sample grasp positions that
    need to be evaluated later on to see if they are
    feasible.
    """
    sphere_radius = .005

    # Randomly sample points from the point cloud
    sample_indices = np.random.choice(len(points), size=num_samples, replace=False)
    sampled_points = points[sample_indices]

    inside_sphere_indices = []

    mat_list = []
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

    rotations = []
    for i in range(len(mat_list)):
        eigenvalues, eigenvectors = np.linalg.eig(mat_list[i])

        # Sort eigenvalues and corresponding eigenvectors in ascending order
        sorted_indices = np.argsort(eigenvalues)
        sorted_eigenvectors = eigenvectors[:, sorted_indices]
        rotations.append(sorted_eigenvectors)

    # Only take grasp poses that follow these two rules
    #  - No points can be inside of the gripper
    #  - There have to be points in between the gripper's fingers

    if verbose:
        u = sorted_eigenvectors[:, 0]
        v = sorted_eigenvectors[:, 1]
        w = np.cross(u, v)
        w = w / np.linalg.norm(w)  


        origin = [sampled_points[i][0], sampled_points[i][1], sampled_points[i][2]]

        scale_factor = 0.07
        vertices = np.array([
            origin,
            u * scale_factor + origin,
            v * scale_factor + origin,
            w * scale_factor + origin
        ])

        lines = [[0, 1], [0, 2], [0, 3]]

        frame_mesh = o3d.geometry.LineSet()
        frame_mesh.points = o3d.utility.Vector3dVector(vertices)
        frame_mesh.lines = o3d.utility.Vector2iVector(lines)

        colors = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
        frame_mesh.colors = o3d.utility.Vector3dVector(colors)

        o3d.visualization.draw_geometries([point_cloud, frame_mesh],
                                        window_name="Custom Coordinate Frame with Colored Axes",
                                        width=800, height=600)
    
    return sampled_points, rotations
