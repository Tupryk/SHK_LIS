import robotic as ry
import numpy as np
from utils import *
import matplotlib.pyplot as plt
import rowan 

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingleWithTopCamera.g'))

midpoint = [-0.105, 0.2, 0.745]

C.addFrame("blueChest") \
    .setPosition(midpoint+np.array([-.22,.2,0])) \
    .setShape(ry.ST.box, size=[0.21, .36, .15]) \
    .setColor([28/255, 18/255, 210/255]) \

draw_rectangular_arena(C, width=.68, height=.6, center_point=[.19, .32])


C.view(True)
for i in range(4):
    if i > 0:
        C.delFrame("woolBall")

    midpoint=sample_rectangular_arena(width=.68, height=.6, center_point=[.19, .32])

    base_quat = [-1/np.sqrt(2), 1/np.sqrt(2), 0 ,0 ]
    rel_quat = rowan.from_axis_angle([0,1,0], np.random.uniform(0, 2*np.pi))

    C.addFrame("woolBall") \
        .setPosition(midpoint) \
        .setShape(ry.ST.capsule, size=[.08, .07]) \
        .setColor([106/255, 24/255, 79/255]) \
        .setQuaternion(rowan.multiply(base_quat, rel_quat))


    bot = ry.BotOp(C, useRealRobot=False)

    rgb, _, __ = bot.getImageDepthPcl('topCamera')

    plt.imshow(rgb)
    plt.savefig(f"sceneImages/image{i}")
