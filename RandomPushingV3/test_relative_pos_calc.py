from highLevelManipulation import Robot


robot = Robot(real_robot=True)
for i in range(10):
    robot.updateObjectPosition()
    while not robot.pushObject():
        pass
