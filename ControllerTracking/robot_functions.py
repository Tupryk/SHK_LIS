from robotic import ry

def update_robot(ry_config, contoller_pos, bot, controller, contoller_up_vector, contoller_side_vector):

    if controller.rAxis[1].x == 1.:
        bot.gripperClose(ry._left)
    else:
        bot.gripperOpen(ry._left)

    komo = ry.KOMO()
    komo.setConfig(ry_config, True)

    komo.setTiming(1, 1, 1., 0)
    komo.addControlObjective([], 0, 1e-1)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], contoller_up_vector)
    komo.addObjective([], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [1e1], contoller_side_vector)

    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], contoller_pos)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    if ret.feasible:
        path = komo.getPath()
        bot.moveTo(path[0], 1e1, True)
        
    print(ret)
