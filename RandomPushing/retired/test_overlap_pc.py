import robotic as ry
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


# Idea is as followed:
# iterate over pcs viewpoints in steps of 3, so we can select 3 pc viewpoints and artifically
# link the ICP method by adding the middle component to each ICP argument
# so instead of doing ICP(A, B) we do it (A + B, B + C)
# and then merge points with regular ICP i.e., ICP(A, B)
# TO DO: Make the merging with the added link better, due to the length of the list an if statement
# is used in which we perform the older method for some number of pc viewpoints to not get an out of
# bounds error. 
def piece_pci_overlap(pcs):
    while len(pcs) > 1:
        print("Starting loop...")
        # Piece Creation
        new_pcs = []
        for i in range(0, len(pcs), 3):
            new_pcs.append(standardICP(pcs[i] + pcs[i-1], pcs[i-1] + pcs[i-2]))
            if i <= len(pcs) - 3:
                new_pcs.append(standardICP(pcs[i] + pcs[i+1], pcs[i+1] + pcs[i+2]))
            elif i > len(pcs) - 3:
                new_pcs.append(standardICP(pcs[i],  pcs[i+1]))

        pcs = new_pcs

        # Piece Joining
        new_pcs = []
        for i in range(0, len(pcs), 2):
            new_pcs.append(standardICP(pcs[i] , pcs[i+1]))

        pcs = new_pcs
        print("Ending loop...")
    return pcs[0]



point_cloud_name = "blocks"
pcs = [o3d.io.read_point_cloud(f"./data/pcs/{point_cloud_name}/point_cloud_{i}.pcd") for i in range(view_count)]
result = piece_pci(pcs)
o3d.io.write_point_cloud(f"./data/pcs/{point_cloud_name}/pieceWisePC.pcd", result)
result = o3d.io.read_point_cloud(f"./data/pcs/{point_cloud_name}/pieceWisePC.pcd")

pc_overlap_result = piece_pci_overlap(pcs)
o3d.io.write_point_cloud(f"./data/pcs/{point_cloud_name}/pieceWisePCOverlap.pcd", pc_overlap_result)
pc_overlapbuffer_result = o3d.io.read_point_cloud(f"./data/pcs/{point_cloud_name}/pieceWisePCOverlap.pcd")


ry.params_print()
verbose = 0

C = ry.Config()

pc_list = []
# original point cloud
for i in range(view_count):
    v = json.load(open(f"./data/pcs/{point_cloud_name}/point_cloud_{i}.json"))
    pclFrame = C.addFrame(f'pcl{i}')
    pclFrame.setPointCloud(np.array(v))
    pclFrame.setColor([0.,1.,0.])
    pclFrame.setPosition([0,.30,0])
    C.view_recopyMeshes()
    pc_list.append(np.array(v))

combined_point_cloud = np.concatenate(pc_list, axis=0)
print("Orignal PC:", combined_point_cloud.shape)

# piece wise pc
final_points = np.asarray(result.points)
pclFrame = C.addFrame('pcl')
pclFrame.setPointCloud(np.array(final_points))
pclFrame.setColor([0.,0.,1.])
C.view_recopyMeshes()
print("Pairwise PC:", np.array(final_points).shape)

final_points_2 = np.asarray(pc_overlapbuffer_result.points)
pclFrame = C.addFrame('pcl_old')
pclFrame.setPointCloud(final_points_2)
pclFrame.setColor([1.,0.,0.])
pclFrame.setPosition([0,-.30,0.])
print("Pairwise PC with Overlap:", np.array(final_points_2).shape)
C.view_recopyMeshes()

C.view(True)
