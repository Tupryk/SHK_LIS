import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud("./data/pcs/controller/pieceWisePC.pcd")
pcd = pcd.voxel_down_sample(voxel_size=0.005)
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

GRIPPER_FINGER_SEPARATION = .075
GRIPPER_FINGER_WIDTH = .025
"""
    Three things need to be checked for the antipodal points to be valid:

        - The side distance: make a perpendicular vector for one of the point's normals 
        which needs to have a rotation towards the other antipodal point. Trace a line
        through the vector and check the distance, which should be below a certain
        threshhold, like half the width of a gripper finger.

        - The normal's plane distance: Trace a line with the direction of one of the
        points normals and check the distance from said point until the line his the
        plane defined by the other point's normal. The distance should be below a
        threshhold like the gripper finger separation. (Temporarly also set a minimum
        distance for this value)

        - Normals should be looking away from eachother.

        - (Temporary until better point normal extraction) Ignore point with a normal
        that has an angle over 90 degrees in respect to the y axis.

        - Eventually patch contact checking should also be added.
"""

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

# Find antipodal pairs
antipodal_pairs = []
for i in range(len(pcd.points)):
    normal_i = np.asarray(pcd.normals[i])
    for j in range(i + 1, len(pcd.points)):
        normal_j = np.asarray(pcd.normals[j])
        dot_product = np.dot(normal_i, normal_j)
        if (dot_product < -0.7 and
            validSideDistance(pcd.points[i], pcd.points[j], pcd.normals[i]) and
            validNormalDistance(pcd.points[i], pcd.points[j], pcd.normals[i]) and
            validNormalRelativeDir(pcd.points[i], pcd.points[j], pcd.normals[i])
            and validNormalDir(pcd.points[j], pcd.normals[i])
            ):
                antipodal_pairs.append((i, j))

print("Antipodal pairs found:", antipodal_pairs)

# Create a line set to visualize antipodal pairs
line_set = o3d.geometry.LineSet()

# Populate line_set with antipodal pairs
lines = []
for pair in antipodal_pairs:
    lines.append([pair[0], pair[1]])
lines = np.array(lines).astype(np.int32)
line_set.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
line_set.lines = o3d.utility.Vector2iVector(lines)

# Set color of the lines to red
line_colors = np.array([[1, 0, 0] for _ in range(len(lines))])
line_set.colors = o3d.utility.Vector3dVector(line_colors)

# Visualize the point cloud and antipodal pairs
o3d.visualization.draw_geometries([pcd, line_set], point_show_normal=True)
