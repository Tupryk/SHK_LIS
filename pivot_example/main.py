import numpy as np
import robotic as ry

from scene import give_scene


C = give_scene()
C.view(True)

komo = ry.KOMO(C, 2, 1, 2, True)
komo.addControlObjective([], 0, 1e-2)
komo.addControlObjective([], 1, 1e-1)
komo.addControlObjective([], 2, 1e0)
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)

# Grasp the handle
komo.addObjective([1, 2], ry.FS.positionDiff, ["l_gripper", "handle"], ry.OT.eq, [1e1])
# komo.addObjective([1], ry.FS.scalarProductYY, ["table", "l_gripper"], ry.OT.eq, [1e1], [0.])
komo.addObjective([0, 1], ry.FS.qItself, ["hinge"], ry.OT.eq, [1e1], [])

# Open the door
komo.addObjective([2], ry.FS.qItself, ["hinge"], ry.OT.eq, [1e1], [np.pi*.5])

ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()

if not ret.feasible:
    print("KOMO was not solved!")
    exit()

# komo.view(True)
# komo.view_play(True, delay=.5)


# Sub-Motions
(C0, q0, q1) = komo.getSubProblem(0)
pick_komo = ry.KOMO(C0, 1, 32, 2, True)
pick_komo.addControlObjective([], 0, 1e-1)
pick_komo.addControlObjective([], 1, 1e-1)
pick_komo.addControlObjective([], 2, 1e0)
pick_komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
pick_komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
pick_komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e0], [], 1)
pick_komo.initWithWaypoints([q1], 1, interpolate=True, qHomeInterpolate=.5, verbose=0)
pick_komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, scale=[1e0], target=q1)

ret = ry.NLP_Solver(pick_komo.nlp(), verbose=0).solve()

if not ret.feasible:
    print("PICK-KOMO was not solved!")
    exit()

# pick_komo.view(True)
# pick_komo.view_play(True, delay=.5)

(C0, q0, q1) = komo.getSubProblem(1)
pivot_komo = ry.KOMO(C0, 1, 32, 2, True)
pivot_komo.addControlObjective([], 0, 1e-1)
pivot_komo.addControlObjective([], 1, 1e-1)
pivot_komo.addControlObjective([], 2, 1e0)
pivot_komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
pivot_komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
pivot_komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e0], [], 1)
pivot_komo.initWithWaypoints([q1], 1, interpolate=True, qHomeInterpolate=.5, verbose=0)
pivot_komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, scale=[1e0], target=q1)

ret = ry.NLP_Solver(pivot_komo.nlp(), verbose=0).solve()

if not ret.feasible:
    print("PIVOT-KOMO was not solved!")
    exit()

# pivot_komo.view(True)
# pivot_komo.view_play(True, delay=.5)


# Run on robot
bot = ry.BotOp(C, useRealRobot=False)
bot.gripperMove(ry._left)
bot.home(C)
bot.wait(C)

bot.move(pick_komo.getPath(), [3.])
bot.wait(C)

bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C)

bot.move(pivot_komo.getPath(), [3.])
bot.wait(C)

bot.gripperMove(ry._left)
bot.wait(C)
bot.home(C)
