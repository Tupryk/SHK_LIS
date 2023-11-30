import open3d as o3d
import numpy as np
import copy

view_count = 16

threshold = 0.02
iterations = 5
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

combinations = [o3d.io.read_point_cloud(f"./data/point_cloud_{i}.pcd") for i in range(view_count)]
phase = 1
while len(combinations) > 1:
    new_combs = []
    for i in range(0, len(combinations), 2):
        source = combinations[i]
        target = combinations[i+1]
        source, _ = source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        target, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        for j in range(iterations):
            reg_p2p = o3d.pipelines.registration.registration_icp(
                    source, target, threshold, trans_init,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint())
            source.transform(reg_p2p.transformation)
        evaluation = o3d.pipelines.registration.evaluate_registration(source, target,
                                                            threshold, trans_init)
        print(f"Evaluation for combination ({i}, {i+1}): {evaluation}")
        draw_registration_result(source, target, trans_init)
        source += target
        source = source.voxel_down_sample(voxel_size=0.001)
        o3d.io.write_point_cloud(f"./data/phase{phase}combination{i}-{i+1}.pcd", source)
        new_combs.append(source)
    combinations = new_combs
    phase += 1
    print(f"Phase {phase} complete, pcs remaining: ", len(combinations))
