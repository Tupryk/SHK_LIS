### ONLY WORKS FOR BLOCKS

import numpy as np
from highLevelManipulation import Robot

robot = Robot(real_robot=True)

def move_to_table_edge():

    motion_success = False
    while not motion_success:

        table_edge = np.array([robot.obj_pos[0], robot.scan_arena.height*.5 + robot.action_arena.position[1]-.04, 0])
        #action = np.random.choice(["pull", "push"])
        action = np.random.choice(["pull"])
        if action == "push":
            motion_success = robot.pushObject(table_edge)

        elif action == "pull":
            motion_success = robot.pullObject(table_edge)

    return table_edge

# Look towards object and get its center-point
robot.updateObjectPosition()
while True:
    table_edge = move_to_table_edge()
    robot.updateObjectPosition()

    if np.linalg.norm(robot.obj_pos - table_edge) > .05:
        continue

    robot.tableSideGrasp()

# Generate an end pose
# For now this will be relative until we find a method to calculate the relative pose between a block and a block's point cloud
robot.action_arena.randomPointInside()[:2] # the z position can't realy be controlled
# Which face type is up? In other words, what vector is equal to the world z vector
faceUp = np.random.choice(["x", "y", "z"])
# Rotation relative to the z axis
z_rot = np.random.random() * 2*np.pi
