import robotic as ry
import numpy as np
from utils import *

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingleWithTopCamera.g'))

midpoint = [-0.105, 0.2, 0.745]
C.addFrame("woolBall") \
    .setPosition(midpoint) \
    .setShape(ry.ST.capsule, size=[.07, .06]) \
    .setColor([176/255, 39/255, 119/255]) \
    .setQuaternion([-1/np.sqrt(2), 1/np.sqrt(2), 0 ,0 ])

C.addFrame("blueChest") \
    .setPosition(midpoint+np.array([-.2,.1,0])) \
    .setShape(ry.ST.box, size=[0.17, .3, .15]) \
    .setColor([122/255, 120/255, 245/255]) \


C.view(True)
bot = ry.BotOp(C, useRealRobot=False)

rgb, depth, points = bot.getImageDepthPcl('topCamera')

import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10,5))
axs = fig.subplots(1, 2)
axs[0].imshow(rgb)
axs[1].matshow(depth)
plt.show()