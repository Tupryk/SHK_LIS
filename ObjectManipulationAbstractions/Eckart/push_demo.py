import numpy as np
from highManipulation import RobotMan


robot = RobotMan(1)
robot.gripperClose()

for i in ["+x", "-x", "+y", "-y"]:
    robot.pushBlock(pushDirection=i)
    robot.solveAndExecuteProblem()

    robot.moveToPointFreely(np.array([-.3, .04, .8]))
    robot.solveAndExecuteProblem()

robot.C.view(True)
