import copy
import numpy as np
import open3d as o3d


def ICP_matrix(source, target, iterations=10, threshold=.02, trans_init=np.identity(4)):
    source_copy = copy.deepcopy(source)

    cumulative_transformation = np.identity(4)

    for _ in range(iterations):
        source_filtered, _ = source_copy.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        target_filtered, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        source_downsampled = source_filtered.voxel_down_sample(voxel_size=0.001)
        target_downsampled = target_filtered.voxel_down_sample(voxel_size=0.001)

        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_downsampled, target_downsampled, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        cumulative_transformation = np.dot(reg_p2p.transformation, cumulative_transformation)
        source_copy.transform(reg_p2p.transformation)

    return cumulative_transformation

def mstorePCR(point_clouds, smoothing=False, verbose=0):
    result = point_clouds[0]
    
    for i in range(len(point_clouds)-1):
        source, target = point_clouds[i+1], point_clouds[i]
        m = ICP_matrix(source, target)

        source.transform(m)
        result += source

        if smoothing:
            result, _ = result.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            result = result.voxel_down_sample(voxel_size=0.001)

    if verbose:
        o3d.visualization.draw_geometries([result])
    return result
