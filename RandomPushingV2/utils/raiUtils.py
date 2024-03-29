import numpy as np
import robotic as ry


def komoStraightPath(C: ry.Config, komo: ry.KOMO, frames: [str] = [], phases: [int] = [1, 2]) -> ry.KOMO:

    if len(frames) == 2:
        delta = C.getFrame(frames[1]).getPosition() - \
            C.getFrame(frames[0]).getPosition()
        delta /= np.linalg.norm(delta)
        mat = np.eye(3) - np.outer(delta, delta)
        komo.addObjective(phases, ry.FS.positionDiff, [
                          'l_gripper', frames[0]], ry.OT.eq, mat)

        komo.addObjective([phases[0]], ry.FS.positionDiff, [
                            'l_gripper', frames[0]], ry.OT.eq, [1e1])
        komo.addObjective([phases[1]], ry.FS.positionDiff, [
                            'l_gripper', frames[1]], ry.OT.eq, [1e1])

    else:
        raise Exception('Invalid input lengths:')
    
    return komo


def createWaypointFrame(C: ry.Config, name: str, position: np.ndarray, color: [float] = [0., 1., 0.]) -> ry.Frame:
    way = C.getFrame(name)
    if not way:
        way = C.addFrame(name) \
            .setShape(ry.ST.sphere, size=[.01, .002])
    way.setPosition(position).setColor(color)
    return way
