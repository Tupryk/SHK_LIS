import numpy as np
import open3d as o3d


def getPushPoints(points, zLimit=.69, normalMaxZ=.5, verbose=0):
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
    colors = []
    for i in range(len(pcd.points)):
        if pcd.points[i][2] > zLimit and np.abs(pcd.normals[i][2]) < normalMaxZ:
            push_points.append([pcd.points[i], pcd.normals[i]])
            colors.append([0, 1, 0])
        else: colors.append([1, 0, 0])

    if verbose:
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])

    return push_points


def getPokePoints(points, normalMaxZ=.5, verbose=0):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.025, max_nn=20))

    push_points = []
    colors = []
    for i in range(len(pcd.points)):
        if pcd.normals[i][2] >= normalMaxZ:
            push_points.append([pcd.points[i], pcd.normals[i]])
            colors.append([0, 1, 0])
        else: colors.append([1, 0, 0])

    if verbose:
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])

    return push_points
