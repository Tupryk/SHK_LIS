import numpy as np
from highLevelManipulation import Robot
import open3d as o3d

robot = Robot(real_robot=True, object_dimensions=np.array([.12, .04, .04]))

<<<<<<< HEAD
a = robot.updateObjectPosition()

o3d.visualization.draw_geometries([a])
o3d.io.write_point_cloud("block.pcd", a)
=======
robot = Robot(real_robot=True, object_dimensions=np.array([.12, .04, .04]))
for i in range(10):
    robot.updateObjectPosition()
    while not robot.pushObject():
        pass
>>>>>>> ef9027ffc15b9bd953e5f82610fcd91f9e87a580
