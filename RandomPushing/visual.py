import json
import copy
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import robotic as ry
from utils.ransac import point_above_plane, get_plane_from_points


def getFilteredPointCloud(bot, ry_config, arena, z_cutoff=.68):
    bot.sync(ry_config, .0)
    rgb, depth, points = bot.getImageDepthPcl('cameraWrist', False)

    new_rgb = []
    for lines in rgb:
        for c in lines: 
            new_rgb.append(c.tolist())
    rgb = new_rgb

    new_p = []
    for lines in points:
        for p in lines: 
            new_p.append(p.tolist())
    points = new_p

    new_rgb = []
    new_p = []
    for i, p in enumerate(points):
        if sum(p) != 0:
            new_rgb.append(rgb[i])
            new_p.append(p)
    points = np.array(new_p)
    
    cameraFrame = ry_config.getFrame("cameraWrist")

    R, t = cameraFrame.getRotationMatrix(), cameraFrame.getPosition()

    points = points @ R.T
    points = points + np.tile(t.T, (points.shape[0], 1))

    objectpoints=[]
    colors = []
    for i, p in enumerate(points):
        if p[2] > (z_cutoff) and arena.point_in_arena(np.array(p)):
            objectpoints.append(p)
            colors.append(rgb[i])
    return objectpoints, colors

def getObject(bot, ry_config, arena, use_ransac=False):
    """
    Computes center of point cloud from real sense sensor. 

    Args:
        bot (roboter) : 
        ry_config (environment config) :

    Returns:
        Cloud center or middle point (float list): [x, y, z].

    """

    if use_ransac:
        points, _ = getFilteredPointCloud(bot, ry_config, arena, z_cutoff=0)
        normal, plane_point = get_plane_from_points(points, ry_config)
        npoints = []
        for p in points:
            if point_above_plane(p, normal, plane_point):
                npoints.append(p)
        points = npoints
    else:
        points, _ = getFilteredPointCloud(bot, ry_config, arena)

    if not points:
        print("Object not found!")
        return []

    min_coor = np.array([
        min([p[0] for p in points]),
        min([p[1] for p in points]),
        min([p[2] for p in points])
    ])

    max_coor = np.array([
        max([p[0] for p in points]),
        max([p[1] for p in points]),
        max([p[2] for p in points])
    ])

    midpoint = (max_coor+min_coor)/2
    
    pclFrame = ry_config.getFrame("pcl")
    if not pclFrame:
        pclFrame = ry_config.addFrame('pcl')
        pclFrame.setPointCloud(np.array(points))
        pclFrame.setColor([0.,1.,0.]) #only to see it when overlaying with truth
        ry_config.view_recopyMeshes()

        ry_config.addFrame('mid_point') \
            .setPosition(midpoint) \
            .setShape(ry.ST.marker, size=[.2]) \
            .setColor([1, 1, 0])
    else:
        pclFrame.setPointCloud(np.array(points))
        ry_config.view_recopyMeshes()
        ry_config.getFrame('mid_point') \
            .setPosition(midpoint)
    
    return midpoint.tolist()

def lookAtObj(bot, ry_config, objpos):
    
    
    ry_config.getFrame("predicted_obj").setPosition(objpos)
    q_now = ry_config.getJointState()

    # bot.home(ry_config)
    q_home = bot.get_qHome()

    komo = ry.KOMO()
    komo.setConfig(ry_config, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    # Should be predicted obj instead of obj
    komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1.], [.0, .0, .5])
    komo.addObjective([1.], ry.FS.position, ["l_gripper"], ry.OT.ineq, np.array([[.0, .0, -100.]]), [0, 0, 1])
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_home)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)
        
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        key=bot.sync(ry_config, .1)
        if chr(key) == "q":
            print("Terminated (visual)")
            bot.home(ry_config)
            del bot
            del ry_config
            exit()

def plotArena(middleP, innerR, outerR, C, resolution=48):
    '''
    Visualises the pushing area.

    Args: 
        middleP (np.ndarray): The center point of the arena in XYZ coordinates.
        innerR (int): The inner radius of the arena.
        outerR (ZZ): The outer radius of the arena.
        C: The object or container for visualization frames.
        resolution (int, optional): The number of points used to discretize the arena. Default is 48.

    Returns:
        None: Adds visualization frames representing red spheres for the inner and outer area.
    '''
    step_size = 2*np.pi/resolution
    for i in range(resolution):
        angle = step_size*i
        dir_vec = np.array([np.cos(angle), np.sin(angle), 0])
        outer_point = middleP + outerR*dir_vec

        if innerR:
            inner_point = middleP + innerR*dir_vec
            C.addFrame(f'inner_arena_{i}') \
                .setPosition(inner_point) \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor([1, 0, 0])
        
        C.addFrame(f'outer_arena_{i}') \
            .setPosition(outer_point) \
            .setShape(ry.ST.sphere, size=[.02]) \
            .setColor([1, 0, 0])
        
def plotLine(ry_config, start, end, resolution=10):
    seg = end-start
    line_len = np.linalg.norm(seg)
    segsize = line_len / resolution
    seg /= line_len
    seg *= segsize
    for p in range(resolution+1):
        position = start + (seg * p)
        frame = ry_config.getFrame(f"line_{p}")
        if not frame:
            ry_config.addFrame(f"line_{p}") \
                .setPosition(position) \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor([0, 1, 0])
        else:
            frame.setPosition(position)

def scanObject(bot, ry_config, obj_pos, arena, gripper_height=.2, gripper_distance=.2, save_as=None, view_count=16):

    all_points = []
    angle_step = 2*np.pi/view_count
    for i in range(view_count):
        angle = angle_step * i
        new_view = np.array([
            gripper_distance * np.sin(angle),
            gripper_distance * np.cos(angle),
            gripper_height
        ])

        ry_config.getFrame(f'view_point_{i}').setPosition(new_view+obj_pos)
        bot.sync(ry_config)

        komo = ry.KOMO()
        komo.setConfig(ry_config, True)
        komo.setTiming(1., 1, 1., 0)

        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

        komo.addObjective([1.], ry.FS.positionDiff, ['l_gripper', f'view_point_{i}'], ry.OT.eq, [1e1])
        komo.addObjective([1.], ry.FS.vectorZ, ["cameraWrist"], ry.OT.eq, [1.], -new_view/np.linalg.norm(new_view))

        ret = ry.NLP_Solver().setProblem(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        print(ret)
        bot.move(komo.getPath(), [3.])
        while bot.getTimeToEnd() > 0:
            bot.sync(ry_config, .1)

        points, _ = getFilteredPointCloud(bot, ry_config, arena)
        all_points.append([list(p) for p in points])
        getObject(bot, ry_config, arena)

    if save_as:
        json.dump(all_points, open(save_as, "w"))
    
    return all_points

def voxelGridDownsampling(original_pc, voxel_space_dimensions=[10, 10, 10]):
    sampled_pc = []
    voxels = [
        [[[] for _ in range(voxel_space_dimensions[2])]
        for _ in range(voxel_space_dimensions[1])]
        for _ in range(voxel_space_dimensions[0])
    ]

    xs = [p[0] for p in original_pc]
    ys = [p[1] for p in original_pc]
    zs = [p[2] for p in original_pc]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    min_z = min(zs)
    max_z = max(zs)

    for point in original_pc:
        x = int(((point[0] - min_x)/ (max_x - min_x)) * voxel_space_dimensions[0])
        x = x if x != voxel_space_dimensions[0] else voxel_space_dimensions[0]-1
        y = int(((point[1] - min_y)/ (max_y - min_y)) * voxel_space_dimensions[1])
        y = y if y != voxel_space_dimensions[1] else voxel_space_dimensions[1]-1
        z = int(((point[2] - min_z)/ (max_z - min_z)) * voxel_space_dimensions[2])
        z = z if z != voxel_space_dimensions[2] else voxel_space_dimensions[2]-1
        voxels[x][y][z].append(np.array(point))

    for x in range(voxel_space_dimensions[0]):
        for y in range(voxel_space_dimensions[1]):
            for z in range(voxel_space_dimensions[2]):
                if len(voxels[x][y][z]) > 0:
                    sampled_pc.append(sum(voxels[x][y][z])/len(voxels[x][y][z]))

    return sampled_pc


def point2pointPCR(pcd_files, visualize=False):

    # Load the first point cloud as the initial reference
    merged_cloud = o3d.io.read_point_cloud("data/" + pcd_files[0])

    # Define the initial transformation matrix as an identity matrix
    init_transformation = np.identity(4)

    for pcd_file in pcd_files[1:]:
        # Load the next point cloud
        cloud_to_align = o3d.io.read_point_cloud("data/" + pcd_file)

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

    # Save the merged point cloud to a new PCD file
    o3d.io.write_point_cloud('merged_point_cloud.pcd', merged_cloud)

    # Load the PCD file
    point_cloud = o3d.io.read_point_cloud("merged_point_cloud.pcd")

    points = np.asarray(point_cloud.points)
    if visualize:
        # Extract the X, Y, and Z coordinates
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # Create a 2D scatter plot
        plt.figure(figsize=(10, 10))
        plt.scatter(x, y, s=0.1)  # Adjust the 's' parameter to control point size
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('2D Scatter Plot of Point Cloud')
        plt.show()

    return points

def point2pointPCR_normal(pcd_files, visualize=False):

    # Load the first point cloud as the initial reference
    merged_cloud = o3d.io.read_point_cloud("data/" + pcd_files[0])

    # Define the initial transformation matrix as an identity matrix
    init_transformation = np.identity(4)

    for pcd_file in pcd_files[1:]:
        # Load the next point cloud
        cloud_to_align = o3d.io.read_point_cloud("data/" + pcd_file)

        # Compute normals for each point cloud

        cloud_to_align.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.025, max_nn=20))
        merged_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.025, max_nn=20))

        # Perform ICP-Normal registration
        icp_result = o3d.pipelines.registration.registration_icp(
            cloud_to_align, merged_cloud, .02, init_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )

        # Apply the transformation to align the point cloud with the reference
        cloud_to_align.transform(icp_result.transformation)
        print("Transformation:", icp_result.transformation)

        # Combine the aligned point cloud with the merged point cloud
        merged_cloud += cloud_to_align

    # Save the merged point cloud to a new PCD file
    o3d.io.write_point_cloud('merged_point_cloud_normal.pcd', merged_cloud)
    return 0


def standardICP(source, target):
    threshold = 0.02
    iterations = 10
    trans_init = np.asarray([
                            [1., 0., 0., 0.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.]])
    source, _ = source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    target, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    source = source.voxel_down_sample(voxel_size=0.001)
    target = target.voxel_down_sample(voxel_size=0.001)
    for _ in range(iterations):
        reg_p2p = o3d.pipelines.registration.registration_icp(
                source, target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
        source.transform(reg_p2p.transformation)
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

def ICP_matrix(source, target, iterations=10, threshold=.02, trans_init=np.identity(4)):
    source_copy = copy.deepcopy(source)

    cumulative_transformation = np.identity(4)

    for _ in range(iterations):
        source_filtered, _ = source_copy.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        target_filtered, _ = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        source_downsampled = source_filtered.voxel_down_sample(voxel_size=0.001)
        target_downsampled = target_filtered.voxel_down_sample(voxel_size=0.001)

        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_downsampled, target_downsampled, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        cumulative_transformation = np.dot(reg_p2p.transformation, cumulative_transformation)
        source_copy.transform(reg_p2p.transformation)

    return cumulative_transformation

def mstorePCR(point_clouds, smoothing=False, verbose=0):
    result = point_clouds[0]
    
    for i in range(len(point_clouds)-1):
        source, target = point_clouds[i+1], point_clouds[i]
        m = ICP_matrix(source, target)

        source.transform(m)
        result += source

        if smoothing:
            result, _ = result.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            result = result.voxel_down_sample(voxel_size=0.001)

    if verbose:
        o3d.visualization.draw_geometries([result])
    return result

def getPushPoint(bot, ry_config, arena, z_limit=.69, normal_z_max=.5):
    pts, _ = getFilteredPointCloud(bot, ry_config, arena)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.025, max_nn=20))

    push_points = []
    for i in range(len(pcd.points)):
        if pcd.points[i][2] > z_limit and np.abs(pcd.normals[i][2]) < normal_z_max:
            push_points.append([pcd.points[i], pcd.normals[i]])

    return push_points
