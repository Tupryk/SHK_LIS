import robotic as ry
import numpy as np
import time

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

bot = ry.BotOp(C, useRealRobot=False)

bot.gripperMove(ry._left, .0)

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

# hard coded waypoints tested for giraffe: {X:"t(.6 .1 1.)", mesh: <giraffe.ply>, mass:.1, inertia:[0.0016031026, 0.00129724411 , 0.000385311293], contact: 1}
C.addFrame('way1'). setShape(ry.ST.marker, [.1]) .setPosition([.2, .1, .67])
C.addFrame('way2'). setShape(ry.ST.marker, [.1]) .setPosition([.6, 0, .67])



qHome = C.getJointState()
komo = ry.KOMO(C, 2, 20, 2, False)

komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e-1)

komo.addObjective([1,2], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0,0,1])
komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', 'way1'], ry.OT.eq, [1e1])
komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', 'way2'], ry.OT.eq, [1e1])


ret = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
print(ret)

q = komo.getPath()
C.view(True)
print(type(q), len(q))
bot.moveAutoTimed(q, .7)

while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

del komo #also closes komo view
