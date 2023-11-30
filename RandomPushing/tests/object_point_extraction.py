from robotic import ry
import json
import numpy as np
from config import setup_config, startup_robot
from random_paths import generate_waypoints, compute_motion, run_waypoints_one_by_one
from RandomPushing.utils.bird_search import flyToPoint, getBirdView
from visual import getObject
from time import sleep

WAYPOINTS = 6
INITIAL_OBJ_POS = [-.50, .1, .69]
OBJ_HEIGHT = .08

if __name__ == "__main__":

    #-- load parameters, typically automatically from 'rai.cfg'
    ry.params_print()

    #-- define a configuration
    C = setup_config(WAYPOINTS, INITIAL_OBJ_POS)

    bot = startup_robot(C)

    bot.home(C)

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    # Robot gripper has to be looking down
    opos = C.getFrame("obj").getPosition()
    gpos = C.getFrame("l_gripper").getPosition()
    komo.addObjective([1.], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], (opos-gpos)*-1)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)
        
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)

    getObject(bot, C)

    C.view(True)
