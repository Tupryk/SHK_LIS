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
                if x == 0 or y == 0 or z == 0 or x == x_count-1 or y == y_count-1 or z == z_count-1:
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
        point_cloud, synthetic, max_correspondence_distance=0.1,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=50)
    )

    transformation_matrix = icp_result.transformation

    if verbose:
        print(transformation_matrix)
        old_pc = copy.deepcopy(point_cloud)
        point_cloud.transform(transformation_matrix)
        o3d.visualization.draw_geometries([point_cloud, synthetic, old_pc])

    return np.linalg.inv(transformation_matrix)
