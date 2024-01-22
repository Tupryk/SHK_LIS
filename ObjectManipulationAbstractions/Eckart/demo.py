import numpy as np
import robotic as ry
from basicAbstractions import *

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

C.addFrame('block') \
    .setPosition([-.15,.1,.67]) \
    .setShape(ry.ST.ssBox, size=[.06,.12,.06,.002]) \
    .setColor([1,.5,0]) \
    .setContact(True) \
    .setMass(1e-2)

bot = ry.BotOp(C, False)
bot.home(C)

C.view()

moveObjectBy(C, bot, "block", np.array([-.15,.05,.03]), np.array([0.,0.,np.pi*.5]))
bot.home(C)

C.view(True)
