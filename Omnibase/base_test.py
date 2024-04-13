import robotic as ry
import numpy as np

C = ry.Config()
ry.params_print()
C.addFile('./g_files/scenarios/pandaSingle.g')
# C.addFile('./g_files/scenarios/pandaOmnibase.g')
# C.addFile('./g_files/omnibase/omnibase.g')

bot = ry.BotOp(C, True)

bot.home(C)
exit()

C.view(True)

def getPath(C: ry.Config):
    komo = ry.KOMO(C, 3., 20, 2, False)
    komo.addControlObjective([], 2, 1.)

    komo.addObjective([1.], ry.FS.positionDiff, ["omnibase", "way1"], ry.OT.eq, [1e2])
    komo.addObjective([2.], ry.FS.positionDiff, ["omnibase", "way2"], ry.OT.eq, [1e2])
    komo.addObjective([1.,2.], ry.FS.vectorX, ["omnibase"], ry.OT.eq, [1e2], [0, 1, 0])
    komo.addObjective([3.], ry.FS.positionDiff, ["omnibase", "way0"], ry.OT.eq, [1e2])
    komo.addObjective([3.], ry.FS.vectorX, ["omnibase"], ry.OT.eq, [1e2], [1, 0, 0])
    komo.addObjective([3.], ry.FS.qItself, [], ry.OT.eq, [1e2], [], 1)
    # komo.addObjective([1.], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e2], C.getFrame("l_gripper").getPosition() + np.array([0, 0, -.2]))
    # komo.addObjective([2.], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e2], C.getFrame("l_gripper").getPosition() + np.array([0, 0, .2]))
    # komo.addObjective([3.], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e2], C.getFrame("l_gripper").getPosition())

    # komo.addObjective([], ry.FS.position, ["omnibase"], ry.OT.eq, [1e2], C.getFrame("omnibase").getPosition())
    # komo.addObjective([], ry.FS.quaternion, ["omnibase"], ry.OT.eq, [1e2], C.getFrame("omnibase").getQuaternion())

    sol = ry.NLP_Solver()
    sol.setProblem(komo.nlp())
    sol.setOptions(damping=1e-3, verbose=0, stopTolerance=1e-3, maxLambda=100., stopEvals=200)
    ret = sol.solve()
    return komo.getPath()

C.addFrame("way0").setPosition([0., 0., .1])
C.addFrame("way1").setPosition([1., 0., .1])
C.addFrame("way2").setPosition([0., 1., .1])
C.view(False)

q = getPath(C)

bot.move(q, [5.])
bot.wait(C)
