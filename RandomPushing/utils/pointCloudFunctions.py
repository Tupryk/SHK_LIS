import numpy as np
import open3d as o3d
GRIPPER_FINGER_SEPARATION = .075
GRIPPER_FINGER_WIDTH = .025


def validSideDistance(p1, p2, n):
    dist = np.linalg.norm(p2-p1)
    normal_dist = abs(np.dot(p1 - p2, n) / np.linalg.norm(n))
    side_dist = np.sqrt(dist*dist-normal_dist*normal_dist)
    return side_dist <= GRIPPER_FINGER_WIDTH*.5

def validNormalDistance(p1, p2, n, dist_thresh=GRIPPER_FINGER_SEPARATION):
    return (abs(np.dot(p1 - p2, n) / np.linalg.norm(n)) <= dist_thresh
            and abs(np.dot(p1 - p2, n) / np.linalg.norm(n)) >= .025)

def validNormalRelativeDir(p1, p2, n):
    p2pvec = p2-p1
    dot_product = np.dot(n, p2pvec)
    # Normal vector has magnitude one so we ignore it
    magnitude_vector = np.linalg.norm(p2pvec)
    cosine_angle = dot_product / magnitude_vector
    return cosine_angle <= 0

def validNormalDir(n1, n2):
    upvec = np.array([0., 0., 1.])
    dot_product1 = np.dot(n1, upvec)
    dot_product2 = np.dot(n2, upvec)
    return dot_product1 >= 0 and dot_product2 >= 0

def inPatches(pcd, pIndex, equal_normal_thresh=.7, planar_distance_thresh=.002, dist=GRIPPER_FINGER_WIDTH*2/3):
    pcd_points = np.asarray(pcd.points)
    point = pcd_points[pIndex]

    distances = np.linalg.norm(pcd_points - point, axis=1)

    within_distance_indices = np.where(distances <= dist)[0]

    pNormal = np.asarray(pcd.normals[pIndex])

    on_plane = []
    D = -(np.dot(pNormal, point))
    for i in within_distance_indices:
        plane_distance = np.abs(np.dot(pNormal, pcd_points[i]) + D) / np.linalg.norm(pNormal)
        if plane_distance <= planar_distance_thresh:
            on_plane.append(i)

    patch_points = []
    for i in on_plane:
        comp = np.asarray(pcd.normals[i])
        if np.dot(pNormal, comp) > equal_normal_thresh:
            patch_points.append(i)

    return patch_points

def calculate_valid_antipodal_pairs(pcd, min_patch_points=15, equal_normal_thresh=.9, verbose=0):

    # Prepare point cloud
    pcd = pcd.voxel_down_sample(voxel_size=.005)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.025, max_nn=20))

    if verbose:
        print(f"Finding possible grasp positions for point cloud with {len(pcd.points)} points.")

    # Find points in patches
    points_in_patches = []
    for i in range(len(pcd.points)):
        patch = inPatches(pcd, i, equal_normal_thresh=equal_normal_thresh)
        if len(patch) >= min_patch_points:
            points_in_patches.append(i)
            
            if verbose > 2:
                patch = np.array(patch)
                num_points = np.asarray(pcd.points).shape[0]
                colors = np.tile([0, 0, 1], (num_points, 1))
                colors[patch] = [1, 0, 0]
                pcd.colors = o3d.utility.Vector3dVector(colors)
                o3d.visualization.draw_geometries([pcd], point_show_normal=True)


    # Find antipodal pairs
    antipodal_pairs = []
    for i in range(len(points_in_patches)):
        p1 = points_in_patches[i]
        normal_i = np.asarray(pcd.normals[p1])
        for j in range(i + 1, len(points_in_patches)):
            p2 = points_in_patches[j]
            normal_j = np.asarray(pcd.normals[p2])
            dot_product = np.dot(normal_i, normal_j)

            if ((dot_product < -equal_normal_thresh or dot_product > equal_normal_thresh) and
                validSideDistance(pcd.points[p1], pcd.points[p2], pcd.normals[p1]) and
                validNormalDistance(pcd.points[p1], pcd.points[p2], pcd.normals[p1]) and
                validNormalRelativeDir(pcd.points[p1], pcd.points[p2], pcd.normals[p1])
                and validNormalDir(pcd.points[p2], pcd.normals[p1])
                ):
                    antipodal_pairs.append((p1, p2))
        
        if verbose and (i+1)%100 == 0:
            print(f"Searching Possible Grasps... (Searched {((i+1)/len(pcd.points)*100):.2f}, Total Points: {len(pcd.points)})")

    if verbose:
        print("Antipodal pairs found:", antipodal_pairs)

    if verbose > 1:
        line_set = o3d.geometry.LineSet()

        lines = []
        for pair in antipodal_pairs:
            lines.append([pair[0], pair[1]])
        lines = np.array(lines).astype(np.int32)
        line_set.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
        line_set.lines = o3d.utility.Vector2iVector(lines)

        line_colors = np.array([[1, 0, 0] for _ in range(len(lines))])
        line_set.colors = o3d.utility.Vector3dVector(line_colors)

        o3d.visualization.draw_geometries([pcd, line_set], point_show_normal=True)

    return antipodal_pairs
