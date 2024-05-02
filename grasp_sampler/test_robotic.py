
import pygame
import numpy as np
import open3d as o3d
import robotic as ry


pcd_path = "registrated.pcd"
point_cloud = o3d.io.read_point_cloud(pcd_path)
point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.1, max_nn=30))
points = np.asarray(point_cloud.points)
normals = np.asarray(point_cloud.normals)

C = ry.Config()
# C.addFile("justGripper.g")
C.getJointState() # This needs to be done in order for C.eval to work

C.addFrame("grasp_area") \
    .setShape(ry.ST.box, [.05, .05, .05]) \
    .setPosition([.2, .2, .65]) \
    .setColor([1., .0, .0, .5])

C.addFrame('pcl') \
    .setPointCloud(points) \
    .setColor([0., 1., 0.])
C.view_recopyMeshes()

C.addFrame("point_frame")

C.view(True)

point_in_box_indices = []
for i, point in enumerate(points):
    C.getFrame("point_frame").setPosition(point)
    result, _ = C.eval(ry.FS.negDistance, ["point_frame", "grasp_area"])
    if result[0] >= 0:
        point_in_box_indices.append(i)

points = points[point_in_box_indices]
normals = normals[point_in_box_indices]

C0 = ry.Config()

C0.addFrame('pcl') \
    .setPointCloud(points) \
    .setColor([0., 1., 0.])
C0.view_recopyMeshes()

C0.view(True)

voxel_dim = 16
grasp_area_dims = C.getFrame("grasp_area").getSize()
voxel_area_dims = grasp_area_dims / voxel_dim
voxel_offset = voxel_area_dims*.5 - grasp_area_dims*.5

point_frame = C.getFrame("point_frame")
voxel = C.addFrame("voxel", "grasp_area").setShape(ry.ST.box, voxel_area_dims)

voxels = [
        [[0. for _ in range(voxel_dim)]
        for _ in range(voxel_dim)]
        for _ in range(voxel_dim)
    ]

voxels_normals = [
        [[0. for _ in range(voxel_dim)]
        for _ in range(voxel_dim)]
        for _ in range(voxel_dim)
    ]

max_density = 0
for x in range(voxel_dim):
    for y in range(voxel_dim):
        for z in range(voxel_dim):

            density = 0
            normal_sum = np.array([0., 0., 0.])
            vox_pos = np.array([x*voxel_area_dims[0], y*voxel_area_dims[1], z*voxel_area_dims[2]]) + voxel_offset
            voxel.setRelativePosition(vox_pos)

            for i, point in enumerate(points):
                point_frame.setPosition(point)
                result, _ = C.eval(ry.FS.negDistance, ["point_frame", "voxel"])
                if result[0] >= 0:
                    density += 1
                    normal_sum += normals[i]

            voxels[x][y][z] = density
            if density > 0:
                voxels_normals[x][y][z] = normal_sum / density
            if density > max_density: max_density = density
    print(f"{((x+1)/voxel_dim*100):.2f}% Done")

pygame.init()

window_size = 400
surface = pygame.display.set_mode((window_size, window_size))
square_size = window_size/voxel_dim
voxels_plane = [
        [0. for _ in range(voxel_dim)]
        for _ in range(voxel_dim)
    ]
voxels_plane_normals = [
        [0. for _ in range(voxel_dim)]
        for _ in range(voxel_dim)
    ]

max_density = 0
for i in range(voxel_dim):
    for j in range(voxel_dim):
        density = 0
        normal = np.array([0., 0., 0.])
        for k in range(voxel_dim):
            density += voxels[i][j][k]
            normal += voxels_normals[i][j][k]
        voxels_plane[i][j] = density
        if np.linalg.norm(normal) > 0:
            voxels_plane_normals[i][j] = normal / np.linalg.norm(normal)
        if density > max_density: max_density = density

for i in range(voxel_dim):
    for j in range(voxel_dim):
        c = voxels_plane[i][j]/max_density*255
        color = (c, c, c)
        pygame.draw.rect(surface, color, pygame.Rect(i*square_size, j*square_size, square_size, square_size))

pygame.display.flip()
running=True
while running:
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False

for i in range(voxel_dim):
    for j in range(voxel_dim):
        if np.linalg.norm(voxels_plane_normals[i][j]) > 0:
            color = (voxels_plane_normals[i][j] + np.array([1., 1., 1.])) * .5 * 255
            pygame.draw.rect(surface, color, pygame.Rect(i*square_size, j*square_size, square_size, square_size))

pygame.display.flip()
running=True
while running:
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False
pygame.quit()
