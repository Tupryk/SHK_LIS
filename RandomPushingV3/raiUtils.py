import numpy as np
import robotic as ry
from typing import List


def komoStraightPath(C: ry.Config, komo: ry.KOMO, frames: List[str]=[], phases: List[int]=[1, 2], gotoPoints: bool=True) -> ry.KOMO:

    if len(frames) == 2:
        delta = C.getFrame(frames[1]).getPosition() - \
            C.getFrame(frames[0]).getPosition()
        delta /= np.linalg.norm(delta)
        mat = np.eye(3) - np.outer(delta, delta)
        komo.addObjective(phases, ry.FS.positionDiff, [
                          'l_gripper', frames[0]], ry.OT.eq, mat)

        if gotoPoints:
            komo.addObjective([phases[0]], ry.FS.positionDiff, [
                                'l_gripper', frames[0]], ry.OT.eq, [1e1])
            komo.addObjective([phases[1]], ry.FS.positionDiff, [
                                'l_gripper', frames[1]], ry.OT.eq, [1e1])

    else:
        raise Exception('Invalid input lengths:')
    
    return komo


def createWaypointFrame(C: ry.Config, name: str, position: np.ndarray, color: List[float] = [1., 0., 1.]) -> ry.Frame:
    way = C.getFrame(name)
    if not way:
        way = C.addFrame(name) \
            .setShape(ry.ST.sphere, size=[.01, .005]) \
            .setContact(False)
    way.setPosition(position).setColor(color)
    return way


def setupConfig(on_real: bool=False,
                obj_pos: np.ndarray=np.array([-.5, .1, .69])) -> ry.Config:

    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
    
    if not on_real:
        C.addFrame('obj') \
            .setPosition(obj_pos) \
            .setShape(ry.ST.ssBox, [.12, .12, .04, 0]) \
            .setColor([1, .5, 0]) \
            .setMass(.1)
    
    return C


def startupRobot(C: ry.Config, on_real: bool) -> ry.BotOp:

    bot = ry.BotOp(C, on_real)
    bot.home(C)

    bot.gripperMove(ry._left, .01, .4)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    
    return bot


def basicKomo(C: ry.Config, phases: int=1, slices: int=20, enableCollisions: bool=True) -> ry.KOMO:

        q_now = C.getJointState()

        komo = ry.KOMO()
        komo.setConfig(C, enableCollisions)
        komo.setTiming(phases, slices, 1., 2)

        komo.addControlObjective([], 1, 1e-1)
        komo.addControlObjective([], 2, 1e0)

        if enableCollisions:
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)

        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
        komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

        komo.addObjective([phases], ry.FS.qItself, [], ry.OT.eq, [10.], [], 1)
        
        return komo

def plotLine(C: ry.Config,
             start: np.ndarray,
             end: np.ndarray,
             name: str="line",
             resolution: float=10.,
             color: List[float]=[1., 0., 0.]):

    seg = end-start
    line_len = np.linalg.norm(seg)
    segsize = line_len / resolution
    seg /= line_len
    seg *= segsize

    for p in range(resolution+1):
        position = start + (seg * p)
        frame = C.getFrame(f"{name}_{p}")
        if not frame:
            C.addFrame(f"{name}_{p}") \
                .setPosition(position) \
                .setShape(ry.ST.sphere, size=[.005]) \
                .setColor(color)
        else:
            frame.setPosition(position)
