from highLevelManipulation import Robot

robot = Robot(real_robot=True)

push_attempts = 100
for i in range(push_attempts):

    # Look towards object and get its center-point
    robot.updateObjectPosition()

    # Starting from home makes push calculation easier
    robot.goHome()

    # Try to calculate push motions in different directions until you find a feasible push motion
    while not robot.pushObject(save_as=f"./push_attempt_{i+1+213}.json"):
        pass

    print("Achieved push number ", i+1)
