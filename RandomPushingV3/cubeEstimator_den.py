import copy
import numpy as np
import open3d as o3d

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(source, target, voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    trans_init = np.eye(4)
    source.transform(trans_init)

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return  source_down, target_down, source_fpfh, target_fpfh



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

    voxel_size = 0.002  
    source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(point_cloud, synthetic,
    voxel_size)
    result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                               voxel_size)
    draw_registration_result(source_down, target_down, result_ransac.transformation)
    trans_init = np.asarray(result_ransac.transformation)
        # Perform icp to get the pose of the point cloud with respect to the original position
    threshold = 10
    trans_init = np.asarray(result_ransac.transformation)

    print("Apply point-to-point ICP")
    icp_result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(icp_result)
    print("Transformation is:")
    print(icp_result.transformation)
    draw_registration_result(source_down, target_down, icp_result.transformation)

    transformation_matrix = icp_result.transformation
    if verbose:
        print(transformation_matrix)
        old_pc = copy.deepcopy(point_cloud)
        point_cloud.transform(transformation_matrix)
        o3d.visualization.draw_geometries([point_cloud, synthetic, old_pc])

    return np.linalg.inv(transformation_matrix)
