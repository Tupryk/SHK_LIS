from highManipulation import RobotMan


robot = RobotMan()
robot.gripperClose()

for i in ["+x", "-x", "+y", "-y"]:
    robot.pushBlock(push_direction=i)

    robot.moveToPointZLock([-.3, .04, .8])

robot.C.view(True)
