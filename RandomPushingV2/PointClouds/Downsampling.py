import numpy as np


def pointNormalVoxels(pointCloud, voxelSize=.005):

    # Limits
    min_coor = np.array([
        min([p[0] for p in pointCloud.points]),
        min([p[1] for p in pointCloud.points]),
        min([p[2] for p in pointCloud.points])
    ])
    max_coor = np.array([
        max([p[0] for p in pointCloud.points]),
        max([p[1] for p in pointCloud.points]),
        max([p[2] for p in pointCloud.points])
    ])

    # Prepare Voxels
    dims = max_coor-min_coor
    voxelSpaceDims = np.ceil(dims/voxelSize)
    voxelSpaceDims = voxelSpaceDims.astype(int)
    voxels = [
        [[[] for _ in range(voxelSpaceDims[2])]
        for _ in range(voxelSpaceDims[1])]
        for _ in range(voxelSpaceDims[0])
    ]

    # Voxel Assigning
    for i, point in enumerate(pointCloud.points):
        x = int(((point[0] - min_coor[0])/ dims[0]) * voxelSpaceDims[0])
        x = x if x != voxelSpaceDims[0] else voxelSpaceDims[0]-1
        y = int(((point[1] - min_coor[1])/ dims[1]) * voxelSpaceDims[1])
        y = y if y != voxelSpaceDims[1] else voxelSpaceDims[1]-1
        z = int(((point[2] - min_coor[2])/ dims[2]) * voxelSpaceDims[2])
        z = z if z != voxelSpaceDims[2] else voxelSpaceDims[2]-1
        voxels[x][y][z].append([point, pointCloud.normals[i]])

    return voxels, voxelSpaceDims

def normalVoting(normals, outlierThresh=3.):
    # Outlier detection
    mean = np.mean(normals, axis=0)
    std_dev = np.std(normals, axis=0)

    # Avoid invalid divisions
    epsilon = 1e-10
    std_dev += epsilon

    # Remove outliers
    z_scores = np.abs((normals - mean) / std_dev)
    outlier_mask = np.any(z_scores > outlierThresh, axis=1)
    if len(outlier_mask) != len(normals):
        filtered_points = normals[~outlier_mask]
    else: filtered_points = normals

    # Mean of remaining points
    mean_normal = np.mean(filtered_points, axis=0)
    mean_normal /= np.linalg.norm(mean_normal)
    return mean_normal
