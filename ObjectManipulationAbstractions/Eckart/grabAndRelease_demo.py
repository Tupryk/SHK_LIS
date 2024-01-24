from highManipulation import RobotMan


robot = RobotMan()

robot.moveToPointFreely([-.3, .04, .8])
komo = robot.grabBlock("block", "x")

robot.gripperClose()
robot.moveToPointFreely([-.3, .04, .8])
robot.gripperOpen()

robot.C.view(True)
