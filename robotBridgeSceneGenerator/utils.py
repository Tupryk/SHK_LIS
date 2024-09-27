import copy
import numpy as np
import open3d as o3d


def estimate_cube_pose(point_cloud: np.ndarray, dimensions: np.ndarray, add_noise: bool = True, verbose: int = 0, origin=np.array([-.55, -.1, .67])) -> np.ndarray:
    # Generate comparison point cloud from know cube dimensions
    synthetic = []
    step_size = .002
    half_dims = np.array([*dimensions]) * .5
    x_count = int(dimensions[0] / step_size)
    y_count = int(dimensions[1] / step_size)
    z_count = int(dimensions[2] / step_size)
    for x in range(x_count):
        for y in range(y_count):
            for z in range(z_count):
                if x == 0 or y == 0 or x == x_count-1 or y == y_count-1 or z == z_count-1:
                    if add_noise:
                        # Should make this look more like the realsense scan
                        new_point = np.array([x*step_size + np.random.random()*step_size,
                                              y*step_size + np.random.random()*step_size,
                                              z*step_size + np.random.random()*step_size])
                    else:
                        new_point = np.array(
                            [x*step_size, y*step_size, z*step_size])
                    new_point = (new_point + origin - half_dims).tolist()
                    synthetic.append(new_point)

    np_synthetic = np.array(synthetic)
    synthetic = o3d.geometry.PointCloud()
    synthetic.points = o3d.utility.Vector3dVector(np_synthetic)
    if verbose:
        o3d.visualization.draw_geometries([synthetic])
        o3d.visualization.draw_geometries([point_cloud])
        o3d.visualization.draw_geometries([synthetic, point_cloud])

    # Perform icp to get the pose of the point cloud with respect to the original position
    icp_result = o3d.pipelines.registration.registration_icp(
        point_cloud, synthetic, max_correspondence_distance=2.,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=100)
    )

    transformation_matrix = icp_result.transformation

    if verbose:
        print(transformation_matrix)
        old_pc = copy.deepcopy(point_cloud)
        point_cloud.transform(transformation_matrix)
        o3d.visualization.draw_geometries([point_cloud, synthetic, old_pc])

    return np.linalg.inv(transformation_matrix)


def extract_position_and_quaternion(pose_matrix):
    # Extract position (translation) from the last column of the pose matrix
    position = pose_matrix[:3, 3]

    # Extract the rotation matrix from the upper-left 3x3 submatrix
    rotation_matrix = pose_matrix[:3, :3]

    # Compute quaternion from rotation matrix
    trace = np.trace(rotation_matrix)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
    elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
        S = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2  # S = 4 * qx
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
        qx = 0.25 * S
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        S = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2  # S = 4 * qy
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
        qy = 0.25 * S
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
    else:
        S = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2  # S = 4 * qz
        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
        qz = 0.25 * S
    
    quaternion = np.array([qw, qx, qy, qz])

    return position, quaternion

