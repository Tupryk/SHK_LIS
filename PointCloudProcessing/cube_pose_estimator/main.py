import numpy as np
import open3d as o3d
from cubeEstimator import estimate_cube_pose

dimensions = np.array([.12, .12, .04])

add_noise = True
synthetic = []
step_size = .005
x_count = int(dimensions[0] / step_size)
y_count = int(dimensions[1] / step_size)
z_count = int(dimensions[2] / step_size)
for x in range(x_count):
    for y in range(y_count):
        for z in range(z_count):
            if x == 0 or y == 0 or z == 0 or x == x_count-1 or y == y_count-1 or z == z_count-1:
                if add_noise:
                    new_point = [x*step_size + np.random.random()*step_size,
                                    y*step_size + np.random.random()*step_size,
                                    z*step_size + np.random.random()*step_size]
                else:
                    new_point = [x*step_size, y*step_size, z*step_size]
                synthetic.append(new_point)

np_synthetic = np.array(synthetic)
rotation_angle = np.pi / 4
R = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle), 0],
              [np.sin(rotation_angle), np.cos(rotation_angle), 0],
              [0, 0, 1]])
np_synthetic = np.dot(np_synthetic, R.T)

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(np_synthetic)

estimate_cube_pose(point_cloud, dimensions, verbose=1, add_noise=False)
