import robotic as ry
import numpy as np
import open3d as o3d


def getVoxelGrid(original_pc, voxel_widths=[.025, .025, .025]):

    xs = [p[0] for p in original_pc]
    ys = [p[1] for p in original_pc]
    zs = [p[2] for p in original_pc]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    min_z = min(zs)
    max_z = max(zs)

    voxel_space_dimensions = [
        int(np.ceil((max_x-min_x)/voxel_widths[0])),
        int(np.ceil((max_y-min_y)/voxel_widths[1])),
        int(np.ceil((max_z-min_z)/voxel_widths[2]))
    ]

    voxels = [
        [[[]
        for _ in range(voxel_space_dimensions[2])]
        for _ in range(voxel_space_dimensions[1])]
        for _ in range(voxel_space_dimensions[0])
    ]

    for point in original_pc:
        x = int(((point[0] - min_x)/ (max_x - min_x)) * voxel_space_dimensions[0])
        x = x if x != voxel_space_dimensions[0] else voxel_space_dimensions[0]-1
        y = int(((point[1] - min_y)/ (max_y - min_y)) * voxel_space_dimensions[1])
        y = y if y != voxel_space_dimensions[1] else voxel_space_dimensions[1]-1
        z = int(((point[2] - min_z)/ (max_z - min_z)) * voxel_space_dimensions[2])
        z = z if z != voxel_space_dimensions[2] else voxel_space_dimensions[2]-1
        voxels[x][y][z].append(point)

    return voxels, (min_x, min_y, min_z)


def isValidGrip(px, py, pz, voxelGrid, gripShape=[[[0, 0, 0], 0], [[0, 1, 0], 1]]):
    for box in gripShape:
        if box[1]:
            if len(voxelGrid[px+box[0][0]][py+box[0][1]][pz+box[0][2]]) <= 10: return False
        else:
            if len(voxelGrid[px+box[0][0]][py+box[0][1]][pz+box[0][2]]) > 10: return False
    return True

def graspFinder(point_cloud, finger_width=.025, max_finger_open=.075, display=False):
    possibleGrasps = []
    voxels, mins = getVoxelGrid(point_cloud, voxel_widths=[finger_width for _ in range(3)])

    gripCoors = []
    for x, plane in enumerate(voxels):
        for y, line in enumerate(plane):
            for z, points in enumerate(line):
                if y < len(voxels)-4:
                    if isValidGrip(x, y, z, voxels):
                        gripCoors = []
                        gripCoors.append(x)
                        gripCoors.append(y)
                        gripCoors.append(z)
                        break

    if display:
        C = ry.Config()
        pclFrame = C.addFrame('point_cloud_grasping')
        pclFrame.setPointCloud(np.array(point_cloud))
        pclFrame.setColor([0.,0.,1.])
        C.view_recopyMeshes()

        for x, plane in enumerate(voxels):
            for y, line in enumerate(plane):
                for z, points in enumerate(line):
                    voxel = C.addFrame(f"voxel_x{x}y{y}z{z}")
                    voxel.setShape(ry.ST.ssBox, [finger_width for _ in range(3)]+[.005])
                    voxel.setPosition([mins[0] + x*finger_width, mins[1] + y*finger_width, mins[2] + z*finger_width])
                    if len(points) > 10:
                        voxel.setColor([1., 0., 0., .2])
                    else:
                        voxel.setColor([0., 1., 0., .0])
        
        C0 = ry.Config()
        pclFrame = C0.addFrame('point_cloud_grasping')
        pclFrame.setPointCloud(np.array(point_cloud))
        pclFrame.setColor([0.,0.,1.])
        C0.view_recopyMeshes()

        print(gripCoors)

        gripShape=[[[0, 0, 0], 0], [[0, 1, 0], 1]]
        for n, box in enumerate(gripShape):
            voxel = C0.addFrame(f"box_{n}")
            voxel.setShape(ry.ST.ssBox, [finger_width for _ in range(3)]+[.005])
            voxel.setPosition([mins[0] + (gripCoors[0]+box[0][0])*finger_width, mins[1] + (gripCoors[1]+box[0][1])*finger_width, mins[2] + (gripCoors[2]+box[0][2])*finger_width])
            if box[1]:
                voxel.setColor([1., 0., 0., .2])
            else:
                voxel.setColor([0., 1., 0., .2])

        #C.view(True)
        C0.view(True)
    
    return possibleGrasps

pcd = o3d.io.read_point_cloud("./data/pcs/controller/pieceWisePC.pcd")
point_cloud = np.asarray(pcd.points)
graspFinder(point_cloud, display=True)
