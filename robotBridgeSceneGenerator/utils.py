import copy
import numpy as np
import open3d as o3d
import math
import random
import robotic as ry

def sample_arena(a, b, offset=[0,0]):
    """
    sample from the elliptical arena in the z=.745 plane

    Args:
        C: The current robot configuration, representing the kinematic structure as a tree of frames.
        a: Semi-major axis (radius in the x direction)
        b: Semi-minor axis (radius in the y direction)
        offset: Center point of the ellipse
    """
    z_coord = 0.745  # Fixed Z-coordinate

    r = math.sqrt(random.uniform(0, 1))

    angle = random.uniform(0, 2 * math.pi)

    x = offset[0] + r * a * math.cos(angle)
    y = offset[1] + r * b * math.sin(angle)  

    return [x, y, z_coord]


def draw_arena(C, a, b, offset=[0,0], num_points=100):
    """
    Draw equiangular points on the (elliptical) arena in the z=.745 plane into ry.Config C.

    Args:
        C: An optional string for providing additional information or description related to this
                            manipulation instance. Default is an empty string.
        a: Semi-major axis (radius in the x direction)
        b: Semi-minor axis (radius in the y direction)
        offset: Center point of the ellipse
        num_points: Number of points to place along the ellipse.
    """
    z_coord = .745  # Fixed Z-coordinate
    angle_step = 2 * math.pi / num_points  # Fixed angle interval

    for i in range(num_points):
        angle = i * angle_step  # Equiangular spacing
        
        x = offset[0] + a * math.cos(angle)
        y = offset[1] + b * math.sin(angle)
        midpoint = [x, y, z_coord]
        
        C.addFrame(f"point{i}").setPosition(midpoint).setShape(ry.ST.marker, size=[.07]).setColor([1, 0, 0])


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

