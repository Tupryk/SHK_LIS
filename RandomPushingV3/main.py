from highLevelManipulation import Robot

robot = Robot()

push_attempts = 100
for i in range(push_attempts):
    robot.updateObjectPosition()
    robot.pushObject()

robot.displayResults()
