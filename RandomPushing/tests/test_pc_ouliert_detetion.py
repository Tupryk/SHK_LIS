import open3d as o3d
import numpy as np

if __name__ == "__main__":
    source = o3d.io.read_point_cloud("./data/point_cloud_0.pcd")
    cl, ind = source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    #cl, ind = source.remove_radius_outlier(nb_points=6, radius=0.05)
    inlier_cloud = cl
    outlier_cloud = source.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
