import cv2
import rowan
import numpy as np
import robotic as ry
import matplotlib.pyplot as plt
import time

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle_tableCam.g'))

bot = ry.BotOp(C, useRealRobot=False)

p1 = np.array([.0, .1, .89])
p2 = np.array([.0, .4, .99])
p3 = np.array([.0, .25, .75])
p4 = np.array([.1, .4, .8])
p5 = np.array([-.1, .0, .82])
p6 = np.array([.0, .3, .77])


for i in range(1,7):
    C.addFrame(f'way{i}'). setShape(ry.ST.marker, [.1]) .setPosition(list(globals()[f"p{i}"]))


calib_marker = C.addFrame("calib_marker").setShape(ry.ST.sphere, [.0175]).setPosition(p1).setColor([1., .0, .0])

qHome = C.getJointState()

komo = ry.KOMO(C, 6, 1, 0, False)
komo.addObjective(times=[], feature=ry.FS.jointState, frames=[], type=ry.OT.sos, scale=[1e-1], target=qHome)

for i in range(1,7):
    komo.addObjective([i], ry.FS.positionDiff, ['l_gripper', f"way{i}"], ry.OT.eq, [1e1])


ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()
q = komo.getPath()

bot.move(q, [10])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)
