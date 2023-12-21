import numpy as np
import robotic as ry


def pushPCpoint(point, normal, robot_pos, start_dist=.15, end_dist_range=[.1, .15], initial_elevation=.15, config=None):

    robot2point = point-robot_pos
    dir = -1. if np.inner(robot2point, normal) < 0 else 1.
    start_pos = normal * start_dist * dir
    end_dist = np.random.rand() * (end_dist_range[1]-end_dist_range[0]) + end_dist_range[0]
    end_pos = normal * end_dist * dir
    initial = point.copy()+start_pos.copy()
    initial[2] += initial_elevation
    waypoints = [initial, point+start_pos, point, point-end_pos]
    
    if config:
        for i, w in enumerate(waypoints):
            way = config.getFrame(f"way{i}")

            if not way:
                way = config.addFrame(f"way{i}") \
                .setShape(ry.ST.marker, size=[.1])
                if i == 0:
                    way.setColor([1, 0, 0])
                elif i == len(waypoints)-1:
                    way.setColor([0, 0, 1])
                else:
                    way.setColor([0, 1, 0])

            way.setPosition(w)

    return waypoints

def computeKomo(C, bot, ways, verbose=0):

    bot.sync(C, .1)

    komo = ry.KOMO()
    komo.setConfig(C, True)

    komo.setTiming(len(ways), 5, 1., 2)

    move_dir = ways[-1]-ways[1]
    move_dir /= np.linalg.norm(move_dir)

    komo.addControlObjective([], 0, 1e-2)
    komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([1.], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], -move_dir)
    komo.addObjective([1.], ry.FS.scalarProductXZ, ['l_gripper', 'table'], ry.OT.eq, [1e1], [0.])

    for i, way in enumerate(ways):
        komo.addObjective([i+1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], way)

    ret = ry.NLP_Solver().setProblem(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=verbose).solve()
    if verbose: print(ret)
    if verbose > 1 and ret.feasible: komo.view(True)

    return komo.getPath(), ret.feasible
