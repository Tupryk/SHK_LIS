import robotic as ry
import numpy as np
import open3d as o3d

#-- load parameters, typically automatically from 'rai.cfg'
ry.params_print()

#-- define a configuration
C = ry.Config()

pcd = o3d.io.read_point_cloud("merged_point_cloud.pcd")
final_points = np.asarray(pcd.points)

pclFrame = C.addFrame('pcl')
pclFrame.setPointCloud(np.array(final_points))
pclFrame.setColor([0.,0.,1.])
C.view_recopyMeshes()


pcd = o3d.io.read_point_cloud("merged_point_cloud_normal.pcd")
final_points = np.asarray(pcd.points)+np.asarray([.8,0,0])

pclFrame = C.addFrame('pcl2')
pclFrame.setPointCloud(np.array(final_points))
pclFrame.setColor([1.,0.,0])
C.view_recopyMeshes()

C.view(True)