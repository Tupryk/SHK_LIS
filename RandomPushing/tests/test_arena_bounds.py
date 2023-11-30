#!/usr/bin/env python

from robotic import ry
import numpy as np
import time


ry.params_add({'botsim/verbose': 1., 'physx/motorKp': 10000., 'physx/motorKd': 1000.})
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False, 'this is your workspace data structure C -- NOT THE SIMULTATION')


RECT_ARENA_MIDDLEP=np.array([-.3, -.13, .651])

bot = ry.BotOp(C, False)
#note that in sim, arms are going down! free floating...


# we need to control it somehow, e.g. to home
bot.home(C)
way0 = C.getFrame('start_point')
# t: position relative to cylinder , d: degrees arm rotation, d:degrees gripper rotation
way0.setPosition(RECT_ARENA_MIDDLEP)

way1 = C.getFrame('end_point')
way1.setPosition(RECT_ARENA_MIDDLEP)

bot.moveTo(way1, 2)

while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

bot.home(C)


bot.gripperClose(ry._left)
bot.gripperOpen(ry._left)


bot.sync(C, .0)



del bot
del C



