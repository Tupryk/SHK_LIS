import robotic as ry
import numpy as np
from utils import *
import matplotlib.pyplot as plt
import rowan 

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingleWithTopCamera.g'))

midpoint = [-0.105, 0.2, 0.745]

C.addFrame("blueChest") \
    .setPosition(midpoint+np.array([-.2,.1,0])) \
    .setShape(ry.ST.box, size=[0.17, .3, .15]) \
    .setColor([122/255, 120/255, 245/255]) \

draw_elliptical_arena(C, a=.2, b=.2, center_point=[.05, .25])

for i in range(10):
    if i > 0:
        C.delFrame("woolBall")

    midpoint=sample_elliptical_arena(a=.2, b=.2, center_point=[.05, .25])

    base_quat = [-1/np.sqrt(2), 1/np.sqrt(2), 0 ,0 ]
    rel_quat = rowan.from_axis_angle([0,1,0], np.random.uniform(0, 2*np.pi))

    C.addFrame("woolBall") \
        .setPosition(midpoint) \
        .setShape(ry.ST.capsule, size=[.07, .06]) \
        .setColor([176/255, 39/255, 119/255]) \
        .setQuaternion(rowan.multiply(base_quat, rel_quat))


    C.view(True)

    bot = ry.BotOp(C, useRealRobot=False)

    rgb, _, __ = bot.getImageDepthPcl('topCamera')

    plt.imshow(rgb)
    plt.savefig(f"sceneImages/image{i}")
