import open3d as o3d
from sampler import sample_grasps
from dataset_creator import get_model_input


point_cloud = o3d.io.read_point_cloud("registered.pcd")
sample_points, coor_frames = sample_grasps(point_cloud)

get_model_input(point_cloud, sample_points[0], coor_frames[0], verbose=2)
