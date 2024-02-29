import numpy as np
from highLevelManipulation import Robot


robot = Robot(real_robot=True, object_dimensions=np.array([.12, .04, .04]))
for i in range(10):
    robot.updateObjectPosition()
    while not robot.pushObject():
        pass
