import open3d as o3d
from highLevelManipulation import Robot
from cubeEstimator import estimate_cube_pose


robot = Robot(real_robot=True)
for i in range(10):
    robot.updateObjectPosition()
    while not robot.pushObject():
        pass
