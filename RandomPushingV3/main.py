from highLevelManipulation import Robot

robot = Robot(real_robot=True)

push_attempts = 1
for i in range(push_attempts):
    robot.updateObjectPosition()
    while not robot.pushObject():
        pass
    print("Achieved push number ", i+1)

robot.displayResults()
