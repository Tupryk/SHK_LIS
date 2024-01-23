import numpy as np
from highManipulation import RobotMan


robot = RobotMan(1)

robot.moveToPointFreely(np.array([-.3, .04, .8]))
robot.solveAndExecuteProblem()
komo = robot.grabBlock("block", "x")
robot.solveAndExecuteProblem(verbose=0)

robot.gripperClose()
robot.moveToPointFreely(np.array([-.3, .04, .8]))
robot.solveAndExecuteProblem()
robot.gripperOpen()

robot.C.view(True)
