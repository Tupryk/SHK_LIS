import numpy as np
import robotic as ry
from utils import *


def grabObject(C: ry.Config, objFrameName: str):

    qHome = C.getJointState()

    komo = ry.KOMO(C, 1,1,0, True)
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([], ry.FS.positionDiff, ['l_gripper', objFrameName], ry.OT.eq, [1e1])
    komo.addObjective([], ry.FS.scalarProductXY, ['l_gripper', objFrameName], ry.OT.eq, [1e1], [0])
    komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', objFrameName], ry.OT.eq, [1e1], [0])
    komo.addObjective([], ry.FS.distance, ['l_palm', objFrameName], ry.OT.ineq, [1e1])

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    return komo.getPath(), ret.feasible


def moveGrabbedObject(C: ry.Config,
                      objFrameName: str,
                      finalPos: np.ndarray,
                      finalRot: np.ndarray,
                      dropHeight: float=.01,
                      minMoveHeight: float=.1,
                      verbose: int=0):
    
    # Path direction (Treated as normalized 2d vector)
    initialPos = C.getFrame(objFrameName).getPosition()
    dir = finalPos-initialPos
    dir[2] = 0
    dist = np.linalg.norm(dir)
    dir /= dist
    seg_size = dist/3.

    # Waypoint positions
    height = dist/3.
    if height < minMoveHeight:
        height = minMoveHeight
    finalPos[2] += dropHeight

    seg = dir*seg_size
    point0 = initialPos + seg + np.array([0., 0., height])
    point1 = point0 + seg

    if verbose:
        visualizeWaypoints(C, [initialPos, point0, point1, finalPos])

    komo = ry.KOMO(C, 3, 1, 2, True)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addControlObjective([], 0, 1e-1)
    komo.addControlObjective([], 1, 1e0)

    komo.addObjective([1], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], point0)
    komo.addObjective([2], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], point1)
    komo.addObjective([3], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], finalPos)

    if len(finalRot):
        komo.addObjective([3], ry.FS.quaternion, ['l_gripper'], ry.OT.eq, [1e1], get_quaternion_from_euler(*finalRot))

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    return komo.getPath(), ret.feasible


def moveObjectTo(C: ry.Config,
                 bot: ry.BotOp,
                 objFrameName: str,
                 position: np.ndarray,
                 rotation: np.ndarray = np.array([])):

    pickUpPathathSolution, feasible0 = grabObject(C, objFrameName)
    movementPathSolution, feasible1 = moveGrabbedObject(C, objFrameName, position, rotation, verbose=1)

    if not feasible0 or not feasible1:
        print("Object movement infeasible.")
        return

    bot.move(pickUpPathathSolution, [5])
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)

    bot.gripperMove(ry._left, width=.02, speed=.2)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)

    bot.move(movementPathSolution, [5])
    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)

    bot.gripperMove(ry._left, width=.075, speed=.2)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)


def moveObjectBy(C: ry.Config,
                 bot: ry.BotOp,
                 objFrameName: str,
                 position: np.ndarray,
                 rotation: np.ndarray = np.array([])):
    
    position = C.getFrame(objFrameName).getPosition()+position
    moveObjectTo(C, bot, objFrameName, position, rotation)
