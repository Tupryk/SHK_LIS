import numpy as np
from highLevelManipulation import Robot

robot = Robot(real_robot=True)

robot.updateObjectPosition()

robot.goHome()
print(robot.obj_dims)

robot.graspObject()

robot.moveBack(dir="z")

robot.placeObject(np.array([-.5, -.2]))

robot.moveBack(dir="z")
