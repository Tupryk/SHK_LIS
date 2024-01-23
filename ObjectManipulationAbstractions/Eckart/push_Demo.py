import numpy as np
from highManipulation import RobotMan


robot = RobotMan(1)
robot.gripperClose()

robot.pushBlock()
robot.solveAndExecuteProblem()

robot.moveToPointFreely(np.array([-.3, .04, .8]))
robot.solveAndExecuteProblem()

robot.C.view(True)
