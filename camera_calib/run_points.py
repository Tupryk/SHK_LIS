import cv2
import rowan
import numpy as np
import robotic as ry
import matplotlib.pyplot as plt
import time
import matplotlib.pyplot as plt
from matplotlib.image import imsave

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle_tableCam.g'))

bot = ry.BotOp(C, useRealRobot=True)


p1 = np.array([.0, .1, .89])
p2 = np.array([.15, .4, .9])
p3 = np.array([-.1, .25, .75])
p4 = np.array([.1, .4, .8])
p5 = np.array([-.1, .0, .86])
p6 = np.array([.0, .3, .77])
p7 = np.array([.12, .37, .71])
p8 = np.array([-.1, .33, .72])
p9 = np.array([0, .02, .71])
p10 = np.array([-.1, .2, .76])


for i in range(1,11):
    C.addFrame(f'way{i}'). setShape(ry.ST.marker, [.1]) .setPosition(list(globals()[f"p{i}"]))

C.addFrame(f'wayy'). setShape(ry.ST.marker, [.2]) .setPosition([.40824952, .7, .687])

calib_marker = C.addFrame("calib_marker").setShape(ry.ST.sphere, [.0175]).setPosition(p1).setColor([1., .0, .0])

qHome = C.getJointState()

komo = ry.KOMO(C, 10, 1, 0, False)


komo.addObjective([1,11], ry.FS.scalarProductXX, ['l_gripper', "world"], ry.OT.eq, [1e2])
for i in range(1,11):
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
