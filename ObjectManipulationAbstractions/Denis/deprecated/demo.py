from highManipulation import RobotMan, syncAndExit

robot = RobotMan()
komo = robot.graspTopBox("block", "xy")
robot.executeMotion(komo)
robot.C.view(True)
