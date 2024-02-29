import numpy as np
from highLevelManipulation import Robot
import open3d as o3d

robot = Robot(real_robot=True, object_dimensions=np.array([.12, .04, .04]))

a = robot.updateObjectPosition()

o3d.visualization.draw_geometries([a])
o3d.io.write_point_cloud("block.pcd", a)