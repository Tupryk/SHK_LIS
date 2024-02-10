import open3d as o3d
import numpy as np
import copy 


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
    

source = o3d.io.read_point_cloud(f'../data/point_cloud_{0}.pcd')
target = o3d.io.read_point_cloud(f'../data/point_cloud_{1}.pcd')
transformation = np.matrix("0.942825 , 0.318043 , 0.0996497 , -0.058813; -0.31879 , 0.947785, -0.00874473 , -0.152007; -0.097228 ,-0.0235228,  0.994984 , -0.0458504; 0, 0, 0, 1")
#draw_registration_result(source, target, transformation)

merced =o3d.io.read_point_cloud("../data/result_cloud.pcd")

o3d.visualization.draw_geometries([merced])

merced =o3d.io.read_point_cloud("../data/filtered_result_cloud.pcd")

o3d.visualization.draw_geometries([merced])

merced =o3d.io.read_point_cloud("../data/final_aligned_cloud.pcd")

o3d.visualization.draw_geometries([merced])


merced =o3d.io.read_point_cloud("../data/final_aligned_cloud_filtered.pcd")

o3d.visualization.draw_geometries([merced])