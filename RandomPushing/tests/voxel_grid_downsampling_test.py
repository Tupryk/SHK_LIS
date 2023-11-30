import json
import matplotlib.pyplot as plt
import numpy as np

def voxelGridDownsampling(original_pc, voxel_space_dimensions=[15, 15, 15]):
    sampled_pc = []
    voxels = [
        [[[] for _ in range(voxel_space_dimensions[2])]
        for _ in range(voxel_space_dimensions[1])]
        for _ in range(voxel_space_dimensions[0])
    ]

    # Define limits
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

point_cloud_data = json.load(open("../data/all_point_clouds.json"))
final_pc = voxelGridDownsampling(point_cloud_data)

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(
    [point[0] for point in point_cloud_data],
    [point[1] for point in point_cloud_data],
    [point[2] for point in point_cloud_data], c='b', marker='o')

# Set labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Display the plot
plt.show()

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(
    [point[0] for point in final_pc],
    [point[1] for point in final_pc],
    [point[2] for point in final_pc], c='b', marker='o')

# Set labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Display the plot
plt.show()
