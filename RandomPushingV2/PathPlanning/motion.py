import numpy as np
import robotic as ry


def pushPCpoint(point, normal, robot_pos):
    robot2point = point-robot_pos
    start_vec = normal * (-.05 if np.inner(robot2point, normal) < 0 else .05)
    waypoints = [point-start_vec, point, point+(np.random.rand()*.04+.01)*normal]
    return waypoints

def computeKomo(C, ways, verbose=0):

    komo = ry.KOMO()
    komo.setConfig(C, True)

    komo.setTiming(len(ways), 3, 1., 2)

    move_dir = ways[-1]-ways[0]
    move_dir /= np.linalg.norm(move_dir)

    # komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    # komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], move_dir)
    # komo.addObjective([], ry.FS.vectorX, ['l_gripper'], ry.OT.eq, [1e1], [1., 0., 0.])
    for i, way in enumerate(ways):
        komo.addObjective([i+1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], way)

    ret = ry.NLP_Solver().setProblem(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=verbose).solve()
    if verbose: print(ret)

    return komo.getPath(), ret.feasible
