import open3d as o3d
import numpy as np
import copy

view_count = 32


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


if __name__ == "__main__":
    source = o3d.io.read_point_cloud("../data/point_cloud_0.pcd")
    o3d.visualization.draw_geometries([source])
    for i in range(view_count-1):
        target = o3d.io.read_point_cloud(f"../data/point_cloud_{i+1}.pcd")
        threshold = 0.02
        trans_init = np.asarray([[1., 0., 0., 0.],
                                [0., 1., 0., 0.],
                                [0., 0., 1., 0.], [0., 0., 0., 1.]])
        source, _ = source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        target, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        print("calculating view ", i)
        evaluation = o3d.pipelines.registration.evaluate_registration(source, target,
                                                            threshold, trans_init)
        for _ in range(10):
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source, target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            source.transform(reg_p2p.transformation)
        draw_registration_result(source, target, trans_init)
        source += target
        source = source.voxel_down_sample(voxel_size=0.001)

    o3d.visualization.draw_geometries([source])
