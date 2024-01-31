from highManipulation import RobotMan


robot = RobotMan()
robot.gripperClose()

for i in ["+x", "-x", "+y", "-y"]:
    robot.pushBlock(push_direction=i)

    robot.moveBack()

robot.C.view(True)
