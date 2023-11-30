from robotic import ry
import json
import numpy as np
from config import setup_config, startup_robot
from random_paths import generate_waypointsv2, compute_motion, run_waypoints_one_by_one
from visual import getObject, lookAtObj, plotArena

WAYPOINTS = 6
INITIAL_OBJ_POS = [-.5, 0, .69]
DEBUG = False
OBJ_HEIGHT = .08

INR = None
OTR = .3

ON_REAL = True

robot_pos = np.array([-.5, 0, .651])

if __name__ == "__main__":

    #-- load parameters, typically automatically from 'rai.cfg'
    ry.params_print()
    verbose = 1

    #-- define a configuration
    C = setup_config(WAYPOINTS, INITIAL_OBJ_POS, ON_REAL)
    if DEBUG:
        key = C.view(verbose>0, 'happy with the config?')
        print('key pressed: ', chr(key)) #use this for basic interaction, e.g. aborting the program
        if chr(key)=='q':
            exit()

    bot = startup_robot(C, ON_REAL)

    data = []
    non_f = 0
    obj_pos = INITIAL_OBJ_POS

    # first input to plotArena hardcodes radius pos x,y,z
    plotArena(robot_pos, INR, OTR, C)
    lookAtObj(bot, C, np.array(obj_pos))
    while True:
        obj_pos  = getObject(bot, INR, OTR, robot_pos, C) 
