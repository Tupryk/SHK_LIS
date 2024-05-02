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

# C.view(True)

point_in_box_indices = []
for i, point in enumerate(points):
    C.getFrame("point_frame").setPosition(point)
    result, _ = C.eval(ry.FS.negDistance, ["point_frame", "grasp_area"])
    if result[0] >= 0:
        point_in_box_indices.append(i)

points = points[point_in_box_indices]

# C = ry.Config()

# C.addFrame('pcl') \
#     .setPointCloud(points) \
#     .setColor([0., 1., 0.])
# C.view_recopyMeshes()

# C.view(True)

voxel_dim = 4
grasp_area_dims = C.getFrame("grasp_area").getSize()
voxel_area_dims = grasp_area_dims / voxel_dim
voxel_offset = voxel_area_dims*.5 - grasp_area_dims*.5 + C.getFrame("grasp_area").getPosition()

C = ry.Config()
C.getJointState()
point_frame = C.addFrame("point_frame")

for x in range(voxel_dim):
    for y in range(voxel_dim):
        for z in range(voxel_dim):

            density = 0
            vox_id = f"voxel_{x}_{y}_{z}"
            vox_pos = np.array([x*voxel_area_dims[0], y*voxel_area_dims[1], z*voxel_area_dims[2]]) + voxel_offset
            vox = C.addFrame(vox_id) \
                .setShape(ry.ST.box, voxel_area_dims) \
                .setPosition(vox_pos)

            for point in points:
                point_frame.setPosition(point)
                result, _ = C.eval(ry.FS.negDistance, ["point_frame", f"voxel_{x}_{y}_{z}"])
                if result[0] >= 0:
                    density += 1

            vox.setColor([1., 1., 1., density/6])

C.view(True)
