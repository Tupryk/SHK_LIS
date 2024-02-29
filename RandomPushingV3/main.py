import numpy as np
from highLevelManipulation import Robot

robot = Robot(real_robot=True)

manipulation_attempts = 100
for i in range(manipulation_attempts):

    # Look towards object and get its center-point
    robot.updateObjectPosition()

    # Starting from home makes push calculation easier
    #robot.goHome()

    # Randomly choose a manipulation type
    #action = np.random.choice(["push", "pull", "grasp", "pull_rotate"])
    action = np.random.choice(["pull_rotate"])
    if action == "push":
        # Try to calculate push motions in different directions until you find a feasible push motion
        while not robot.pushObject():
            pass

    elif action == "grasp":
        robot.graspObject()
        robot.placeObject(x_orientation=np.random.choice(["x", "y"]))

    elif action == "pull":
        robot.pullObject()

    elif action == "pull_rotate":
        robot.pullRotateObject(.8)

    print("Achieved manipulation number ", i+1)
