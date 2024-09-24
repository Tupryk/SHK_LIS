import robotic as ry
import numpy as np


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

midpoint = [-0.105, 0.2, 0.745]
C.addFrame("woolBall") \
    .setPosition(midpoint) \
    .setShape(ry.ST.capsule, size=[0.1, .07]) \
    .setColor([176/255, 39/255, 119/255]) \
    .setQuaternion([-1/np.sqrt(2), 1/np.sqrt(2), 0 ,0 ])

C.addFrame("BlueChest") \
    .setPosition(midpoint+np.array([-.2,.1,0])) \
    .setShape(ry.ST.box, size=[0.2, .3, .15]) \
    .setColor([122/255, 120/255, 245/255]) \


# for convenience, a few definitions:
gripper = "l_gripper"
palm = "l_palm"
box = "box1"
table = "table"

C.view(True)