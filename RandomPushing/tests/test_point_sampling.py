import json
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(0)
points_3d = np.array(json.load(open("data/scanned_obj_points.json")))  # 100 random 2D points

# Plotting the original point cloud
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], c='b', marker='o', label='Original Points')
ax.set_title('Original 3D Point Cloud')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.legend()
plt.show()

# Random sampling
random_indices_3d = np.random.choice(points_3d.shape[0], 2_000, replace=False)  # Selecting 30 random points
random_sample_3d = points_3d[random_indices_3d, :]

# Uniform sampling
uniform_indices_3d = np.linspace(0, points_3d.shape[0] - 1, 2_000, dtype=int)  # Selecting 30 uniformly spaced points
uniform_sample_3d = points_3d[uniform_indices_3d, :]

# Displaying the resulting points
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(random_sample_3d[:, 0], random_sample_3d[:, 1], random_sample_3d[:, 2], c='r', marker='o', label='Random Sampled Points')
ax.scatter(uniform_sample_3d[:, 0], uniform_sample_3d[:, 1], uniform_sample_3d[:, 2], c='g', marker='o', label='Uniform Sampled Points')
ax.set_title('Resulting Points from Random and Uniform Sampling in 3D')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.legend()
plt.show()
