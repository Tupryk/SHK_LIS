import numpy as np
import open3d as o3d


def getPushPoints(points, zLimit=.69, normalMaxZ=.5):
    '''
    Args:
        points: Numpy array with points of a point cloud-
        zLimit: This rejects points that are to close to the floor.
        normalMaxZ: This rejects points that have a normal slightly perpendicular
            to the floor. (No downwards or upwards pushing for now)
    
    Returns:
            An array with 2d elements that contain a valid push point in the first index
        and its normal in the second index.
    '''
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.025, max_nn=20))

    push_points = []
    for i in range(len(pcd.points)):
        if pcd.points[i][2] > zLimit and np.abs(pcd.normals[i][2]) < normalMaxZ:
            push_points.append([pcd.points[i], pcd.normals[i]])

    return push_points
