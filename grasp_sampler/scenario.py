import numpy as np
import robotic as ry
from utils import lookAtObj, getFilteredPointCloud

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

INITIAL_OBJ_POS = np.array([-.5, 0, .69])

C.addFrame('obj') \
    .setShape(ry.ST.ssBox, size=[.05, .09, .1, .005]) \
    .setPosition(INITIAL_OBJ_POS) \
    .setColor([1, .5, .5]) \
    .setMass(.1) \
    .setContact(True)

C.addFrame('predicted_obj') \
    .setShape(ry.ST.marker, size=[.02]) \
    .setPosition(INITIAL_OBJ_POS) \
    .setColor([1, .5, .5])

bot = ry.BotOp(C, False)
lookAtObj(bot, C, INITIAL_OBJ_POS)
getFilteredPointCloud(bot, C, INITIAL_OBJ_POS)
C.view(True)
