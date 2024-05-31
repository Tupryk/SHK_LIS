import cv2
import rowan
import numpy as np
import robotic as ry
import matplotlib.pyplot as plt
import time
import matplotlib.pyplot as plt
from matplotlib.image import imsave
from utils import *

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle_tableCam.g'))

bot = ry.BotOp(C, useRealRobot=False)


# p1 = np.array([0, .3, .89])
# p2 = np.array([.15, .4, .9])
# p3 = np.array([-.1, .25, .75])
# p4 = np.array([.1, .4, .8])
# p5 = np.array([-.1, .0, .86])
# p6 = np.array([.0, .3, .77])
# p7 = np.array([.12, .37, .71])
# p8 = np.array([-.1, .33, .72])
# p9 = np.array([0, .02, .71])
# p10 = np.array([-.1, .2, .76])


points3d = sample3dpoints(30, -.15, .15, .05, .4, .7, .9)

for i, p in enumerate(points3d):
    C.addFrame(f'way{i}'). setShape(ry.ST.marker, [.1]) .setPosition(p)

qHome = C.getJointState()

komo = ry.KOMO(C, len(points3d), 1, 0, False)


komo.addObjective([0, len(points3d)], ry.FS.scalarProductXX, ['l_gripper', "world"], ry.OT.eq, [1e2])
for i in range(len(points3d)):
    komo.addObjective([i], ry.FS.positionDiff, ['l_gripper', f"way{i}"], ry.OT.eq, [1e2])
    

ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()
q = komo.getPath()

#print(bot.getCameraFxycxy("cameraFrame"))


bot.gripperMove(ry._left, 0)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

for i, path in enumerate(q):
    bot.moveTo(path)
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)
    rgb, depth, points = bot.getImageDepthPcl('cameraFrame', False)

    imsave(f"photos/image{i}.png", rgb)
