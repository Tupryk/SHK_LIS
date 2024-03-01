import numpy as np
from highLevelManipulation import Robot

robot = Robot(real_robot=True, object_dimensions=np.array([.24, .15, .11]), initial_object_position=np.array([-.55, -.1, .725]))

manipulation_attempts = 100
for i in range(manipulation_attempts):

    # Look towards object and get its center-point
    robot.updateObjectPosition()

    # Starting from home makes push calculation easier
    #robot.goHome()

    # Randomly choose a manipulation type
    #action = np.random.choice(["push", "pull", "grasp", "pull_rotate", "edge_grasp"])
    action = np.random.choice(["pull"])
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

    elif action == "edge_grasp":
        # table_edge = np.array([robot.obj_pos[0], robot.scan_arena.height*.5 + robot.action_arena.position[1] - .04, 0])
        # if robot.pullObject(table_edge):
        #     robot.updateObjectPosition()
        #     if np.linalg.norm(robot.obj_pos - table_edge) > .05:
        #         print("Failed side grasp.")
        #         continue
            robot.tableSideGrasp()

    print("Achieved manipulation number ", i+1)
