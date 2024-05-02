import numpy as np
import open3d as o3d


def sample_grasps(point_cloud, num_samples=1, verbose=0):
    """
    This function gives sample grasp positions that
    need to be evaluated later on to see if they are
    feasible.
    """
    point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.1, max_nn=30))
    sphere_radius = .005

    points = np.asarray(point_cloud.points)
    normals = np.asarray(point_cloud.normals)

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

    for i in range(len(mat_list)):
        eigenvalues, eigenvectors = np.linalg.eig(mat_list[i])

        # Sort eigenvalues and corresponding eigenvectors in ascending order
        sorted_indices = np.argsort(eigenvalues)
        sorted_eigenvectors = eigenvectors[:, sorted_indices]

    # Only take grasp poses that follow these two rules
    #  - No points can be inside of the gripper
    #  - There have to be points in between the gripper's fingers
    
    return sampled_points, sorted_eigenvectors
