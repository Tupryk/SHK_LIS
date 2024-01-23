import numpy as np
from highManipulation import RobotMan


robot = RobotMan(1)

for i in range(1, 4):

    komo = robot.grabBlock(f"block{i}")
    robot.solveAndExecuteProblem(verbose=0)
    robot.gripperClose()

    robot.placeBlock([-.3, .04, .69+(.05*i)])
    robot.solveAndExecuteProblem()
    robot.gripperOpen()

    robot.moveToPointFreely(np.array([-.3, .04, .8]))
    robot.solveAndExecuteProblem()

robot.C.view(True)
