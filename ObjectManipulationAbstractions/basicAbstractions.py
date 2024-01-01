import numpy as np
import robotic as ry


def grabObject(C, frameName):

    qHome = C.getJointState()

    komo = ry.KOMO(C, 1,1,0, True)
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([], ry.FS.positionDiff, ['l_gripper', frameName], ry.OT.eq, [1e1])
    komo.addObjective([], ry.FS.scalarProductXY, ['l_gripper', frameName], ry.OT.eq, [1e1], [0])
    komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', frameName], ry.OT.eq, [1e1], [0])
    komo.addObjective([], ry.FS.distance, ['l_palm', frameName], ry.OT.ineq, [1e1])

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    return komo.getPath(), ret.feasible


def moveGrabbedObject(C, block, finalPos, finalRot, dropHeight=.01):
    initialPos = C.getFrame(block).getPosition()
    dir = finalPos-initialPos
    dist = np.linalg.norm(dir)
    dir /= dist
    height = dist/3
    finalPos[2] += dropHeight

    point0 = initialPos+dir*height + np.array([0., 0., height])
    C.addFrame('way0').setShape(ry.ST.marker, [.1]).setPosition(point0)

    point1 = point0+dir*height
    C.addFrame('way1').setShape(ry.ST.marker, [.1]).setPosition(point1)

    C.addFrame('way2').setShape(ry.ST.marker, [.1]).setPosition(finalPos)

    komo = ry.KOMO(C, 3, 1, 2, True)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    #komo.addControlObjective([], 0, 1e-1)
    #komo.addControlObjective([], 1, 1e0)

    komo.addObjective([1], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], point0)
    komo.addObjective([2], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], point1)
    komo.addObjective([3], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], finalPos)

    komo.addObjective([3], ry.FS.vectorX, ['l_gripper'], ry.OT.eq, [1e1], [1., 0., 0.])
    komo.addObjective([3], ry.FS.vectorY, ['l_gripper'], ry.OT.eq, [1e1], [0., 1., 0.])

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    return komo.getPath(), ret.feasible


def moveObjectTo(C, bot, frameName, position, rotation=None):

    pathSolution, feasible = grabObject(C, frameName)

    if not feasible:
        print("Object movement infeasible.")
        return

    bot.move(pathSolution, [5])
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)

    bot.gripperMove(ry._left, width=.02, speed=.2)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)

    pathSolution, feasible = moveGrabbedObject(C, frameName, position, rotation)

    if not feasible:
        print("Object movement infeasible.")
        return

    bot.move(pathSolution, [5]) # save last pathstep for after loslassing
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)

    bot.gripperMove(ry._left, width=.075, speed=.2)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)

    bot.move(np.asarray([C.getJointState(), pathSolution[-1]]), [1]) # save last pathstep for after loslassing
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)
