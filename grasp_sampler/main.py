import numpy as np
import open3d as o3d
from sampler import sample_grasps
from dataset_creator import get_model_input


pcd_path = "registrated.pcd"
point_cloud = o3d.io.read_point_cloud(pcd_path)
point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.1, max_nn=30))
points = np.asarray(point_cloud.points)
normals = np.asarray(point_cloud.normals)

sample_points, coor_frames = sample_grasps(points, normals, point_cloud)
get_model_input(points, normals, sample_points[0], coor_frames[0], verbose=2)
