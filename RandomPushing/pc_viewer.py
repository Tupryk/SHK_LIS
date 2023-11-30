from robotic import ry
import numpy as np
from visual import point2pointPCR, voxelGridDownsampling
from arena import *
import open3d as o3d
import json

#-- load parameters, typically automatically from 'rai.cfg'
ry.params_print()
verbose = 0

#-- define a configuration
C = ry.Config()

scanned_views = json.load(open("./data/scanned_views.json"))

for i, v in enumerate(scanned_views):
    # v = voxelGridDownsampling(v)
    
    point_cloud_obj = o3d.geometry.PointCloud()
    point_cloud_obj.points = o3d.utility.Vector3dVector(v)
    output_pcd_file = f'data/point_cloud_{i}.pcd'
    o3d.io.write_point_cloud(output_pcd_file, point_cloud_obj)

    pclFrame = C.addFrame(f'pcl{i}')
    pclFrame.setPointCloud(np.array(v))
    pclFrame.setColor([0.,1.,0.])
    pclFrame.setPosition([0,.30,0])
    C.view_recopyMeshes()

#pcd_files = [f'point_cloud_{i}.pcd' for i in range(len(scanned_views))]
#final_points = point2pointPCR(pcd_files, False)

pcd = o3d.io.read_point_cloud("./data/pieceWisePC.pcd")
final_points = np.asarray(pcd.points)

pclFrame = C.addFrame('pcl')
pclFrame.setPointCloud(np.array(final_points))
pclFrame.setColor([0.,0.,1.])
C.view_recopyMeshes()

C.view(True)
