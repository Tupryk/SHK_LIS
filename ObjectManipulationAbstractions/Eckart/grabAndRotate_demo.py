import numpy as np
from highManipulation import RobotMan


robot = RobotMan()

robot.grabBlock(objFrame="long_block")
robot.gripperClose()

aboveBlock = robot.C.getFrame("long_block_objective").getPosition() + np.array([0, 0, .2])
robot.moveToPointZLock(aboveBlock)

# robot.placeBlockRotated("long_block_objective", rotation_type="-x")
robot.placeBlock("long_block_objective", gripperX_aligned="y")
robot.gripperOpen()

robot.moveBack()

robot.goHome()
robot.C.view(True)
