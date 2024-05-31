import cv2
import csv
import numpy as np
import robotic as ry
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from matplotlib.image import imsave
from utils import *

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle_tableCam.g'))

bot = ry.BotOp(C, useRealRobot=True)

points3d = sample3dpoints(25, -.1, .25, .12, .51, .7, .95)

for i, p in enumerate(points3d):
    C.addFrame(f'way{i}'). setShape(ry.ST.marker, [.1]) .setPosition(p)

with open("3dpoints.csv", mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['X', 'Y', 'Z'])  # Write the header

    # Process each image
    for i, _ in enumerate(points3d):
        writer.writerow([i for i in _])

qHome = C.getJointState()

komo = ry.KOMO(C, len(points3d), 1, 0, False)


komo.addObjective([0, len(points3d)], ry.FS.scalarProductXX, ['l_gripper', "world"], ry.OT.eq, [1e2])
for i, _ in enumerate(points3d):
    komo.addObjective([i+1], ry.FS.positionDiff, ['l_gripper', f"way{i}"], ry.OT.eq, [1e2])
    

ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()
q = komo.getPath()


bot.gripperMove(ry._left, 0)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

for i, path in enumerate(q):
    bot.moveTo(path)
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)
    rgb, depth, points = bot.getImageDepthPcl('cameraFrame', False)

    imsave(f"photos/image{i}.png", rgb)

bot.home(C)