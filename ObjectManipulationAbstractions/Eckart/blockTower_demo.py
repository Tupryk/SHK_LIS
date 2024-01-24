import numpy as np
from highManipulation import RobotMan


robot = RobotMan()

for i in range(1, 4):

    komo = robot.grabBlock(f"block{i}")
    robot.gripperClose()

    onTopOf = f"block{i-1}" if i != 1 else "block"
    robot.placeBlock(onTopOf)
    robot.gripperOpen()

    new_starting_pos = robot.C.getFrame("block").getPosition() + np.array([0, 0, .15])
    robot.moveToPointFreely(new_starting_pos)

robot.goHome()
robot.C.view(True)
