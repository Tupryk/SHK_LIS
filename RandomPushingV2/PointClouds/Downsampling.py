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
        voxels[x][y][z].append([np.array(point), pointCloud.normals[i]])

    return voxels
