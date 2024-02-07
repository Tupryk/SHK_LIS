import numpy as np
from highLevelManipulation import Robot

robot = Robot(real_robot=True)

push_attempts = 100
for i in range(push_attempts):

    # Look towards object and get its center-point
    robot.updateObjectPosition()

    # Starting from home makes push calculation easier
    robot.goHome()

    # Randomly choose to grasp or push
    if np.random.random() > .5:
        # Try to calculate push motions in different directions until you find a feasible push motion
        while not robot.pushObject():
            pass
    else:
        robot.graspObject()
        robot.placeObject()



    print("Achieved push number ", i+1)
