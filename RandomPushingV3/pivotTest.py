import numpy as np
from highLevelManipulation import Robot

robot = Robot(real_robot=True, object_dimensions=np.array([.24, .15, .11]), initial_object_position=np.array([-.7, -.1, .725]))
robot.updateObjectPosition()
robot.goHome()
robot.pivot()
