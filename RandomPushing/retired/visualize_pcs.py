import open3d as o3d
import numpy as np
import time
import copy

threshold = 0.02
iterations = 10
trans_init = np.asarray([
                        [1., 0., 0., 0.],
                        [0., 1., 0., 0.],
                        [0., 0., 1., 0.],
                        [0., 0., 0., 1.]])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

source = o3d.io.read_point_cloud(f"./data/difficult_for_pcr/phase3combination0-1.pcd")
target = o3d.io.read_point_cloud(f"./data/difficult_for_pcr/phase3combination2-3.pcd")
def standardICP(source, target):
    source, _ = source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    target, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    source = source.voxel_down_sample(voxel_size=0.001)
    target = target.voxel_down_sample(voxel_size=0.001)
    sourced = source.voxel_down_sample(voxel_size=0.01)
    targetd = target.voxel_down_sample(voxel_size=0.01)
    for j in range(iterations):
        reg_p2p = o3d.pipelines.registration.registration_icp(
                sourced, targetd, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
        sourced.transform(reg_p2p.transformation)
        source.transform(reg_p2p.transformation)
    draw_registration_result(sourced, targetd, trans_init)
    draw_registration_result(source, target, trans_init)
    source += target
    source = source.voxel_down_sample(voxel_size=0.001)
standardICP(source, target)

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

def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    source = o3d.io.read_point_cloud(f"./data/difficult_for_pcr/phase3combination0-1.pcd")
    target = o3d.io.read_point_cloud(f"./data/difficult_for_pcr/phase3combination2-3.pcd")
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

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

"""
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(.01)
result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            .01)
print(result_ransac)
draw_registration_result(source_down, target_down, result_ransac.transformation)


voxel_sizes = o3d.utility.DoubleVector([0.1, 0.05, 0.025])
treg = o3d.t.pipelines.registration

# List of Convergence-Criteria for Multi-Scale ICP:
criteria_list = [
    treg.ICPConvergenceCriteria(relative_fitness=0.0001,
                                relative_rmse=0.0001,
                                max_iteration=20),
    treg.ICPConvergenceCriteria(0.00001, 0.00001, 15),
    treg.ICPConvergenceCriteria(0.000001, 0.000001, 10)
]

# `max_correspondence_distances` for Multi-Scale ICP (o3d.utility.DoubleVector):
max_correspondence_distances = o3d.utility.DoubleVector([0.001, 0.001, 0.001])

# Initial alignment or source to target transform.
init_source_to_target = o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32)

# Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
estimation = treg.TransformationEstimationPointToPlane()

# Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
callback_after_iteration = lambda loss_log_map : print("Iteration Index: {}, Scale Index: {}, Scale Iteration Index: {}, Fitness: {}, Inlier RMSE: {},".format(
    loss_log_map["iteration_index"].item(),
    loss_log_map["scale_index"].item(),
    loss_log_map["scale_iteration_index"].item(),
    loss_log_map["fitness"].item(),
    loss_log_map["inlier_rmse"].item()))

# Setting Verbosity to Debug, helps in fine-tuning the performance.
# o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

s = time.time()

registration_ms_icp = o3d.t.pipelines.registration.registration_multi_scale_icp(source, target, voxel_sizes,
                                           criteria_list,
                                           max_correspondence_distances,
                                           init_source_to_target, estimation,
                                           callback_after_iteration)

ms_icp_time = time.time() - s
print("Time taken by Multi-Scale ICP: ", ms_icp_time)
print("Inlier Fitness: ", registration_ms_icp.fitness)
print("Inlier RMSE: ", registration_ms_icp.inlier_rmse)

draw_registration_result(source, target, registration_ms_icp.transformation)
"""

