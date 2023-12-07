from robotic import ry
import open3d as o3d
import numpy as np
import copy
import json

view_count = 8
threshold = 0.02
iterations = 10
trans_init = np.asarray([
                        [1., 0., 0., 0.],
                        [0., 1., 0., 0.],
                        [0., 0., 1., 0.],
                        [0., 0., 0., 1.]])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def point2pointPCR(pcd_files, path=""):

    # Load the first point cloud as the initial reference
    merged_cloud = o3d.io.read_point_cloud(path + pcd_files[0])

    # Define the initial transformation matrix as an identity matrix
    init_transformation = np.identity(4)

    for pcd_file in pcd_files[1:]:
        # Load the next point cloud
        cloud_to_align = o3d.io.read_point_cloud(path + pcd_file)

        # Perform ICP registration
        icp_result = o3d.pipelines.registration.registration_icp(
            cloud_to_align, merged_cloud, .1, init_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            o3d.pipelines.registration.ICPConvergenceCriteria(1e-6, 1e-6, 30)
        )

        # Apply the transformation to align the point cloud with the reference
        cloud_to_align.transform(icp_result.transformation)
        print("Transformation:", icp_result.transformation)

        # Combine the aligned point cloud with the merged point cloud
        merged_cloud += cloud_to_align

    points = np.asarray(merged_cloud.points)

    return points


def standardICP(source, target, display=False):
    source, _ = source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    target, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    source = source.voxel_down_sample(voxel_size=0.001)
    target = target.voxel_down_sample(voxel_size=0.001)
    for _ in range(iterations):
        reg_p2p = o3d.pipelines.registration.registration_icp(
                source, target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
        source.transform(reg_p2p.transformation)
    if display: draw_registration_result(source, target, trans_init)
    source += target
    source = source.voxel_down_sample(voxel_size=0.001)
    return source

def piece_pci(pcs):
    while len(pcs) > 1:
        print("Starting loop...")
        # Piece Creation
        new_pcs = []
        for i in range(0, len(pcs), 2):
            new_pcs.append(standardICP(pcs[i], pcs[i-1]))
            new_pcs.append(standardICP(pcs[i], pcs[i+1]))

        pcs = new_pcs

        # Piece Joining
        new_pcs = []
        for i in range(0, len(pcs), 2):
            new_pcs.append(standardICP(pcs[i], pcs[i+1]))

        pcs = new_pcs
        print("Ending loop...")
    return pcs[0]

point_cloud_name = "with_table"
pcs = [o3d.io.read_point_cloud(f"./data/pcs/{point_cloud_name}/point_cloud_{i}.pcd") for i in range(view_count)]
result = piece_pci(pcs)
o3d.io.write_point_cloud(f"./data/pcs/{point_cloud_name}/pieceWisePC.pcd", result)
result = o3d.io.read_point_cloud(f"./data/pcs/{point_cloud_name}/pieceWisePC.pcd")
result_old = point2pointPCR([f"point_cloud_{i}.pcd" for i in range(view_count)], path=f"./data/pcs/{point_cloud_name}/")


ry.params_print()
verbose = 0

C = ry.Config()

for i in range(view_count):
    v = json.load(open(f"./data/pcs/{point_cloud_name}/point_cloud_{i}.json"))
    pclFrame = C.addFrame(f'pcl{i}')
    pclFrame.setPointCloud(np.array(v))
    pclFrame.setColor([0.,1.,0.])
    pclFrame.setPosition([0,.30,0])
    C.view_recopyMeshes()

final_points = np.asarray(result.points)
pclFrame = C.addFrame('pcl')
pclFrame.setPointCloud(np.array(final_points))
pclFrame.setColor([0.,0.,1.])
C.view_recopyMeshes()

#final_points_old = np.asarray(result_old.points)
pclFrame = C.addFrame('pcl_old')
pclFrame.setPointCloud(result_old)
pclFrame.setColor([0.,0.,1.])
pclFrame.setPosition([0,-.30,0])
C.view_recopyMeshes()

C.view(True)
