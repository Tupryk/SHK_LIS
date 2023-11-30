from robotic import ry
import numpy as np
from config import setup_config, startup_robot
from visual import lookAtObj, getFilteredPointCloud, getObject
from utils import get_plane_from_points, point_above_plane

WAYPOINTS = 6
INITIAL_OBJ_POS = [-.5, 0, .69]
DEBUG = False
OBJ_HEIGHT = .08

INR = None
OTR = .3

ON_REAL = False

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
    # plotArena(robot_pos, INR, OTR, C)

    lookAtObj(bot, C, np.array(obj_pos))
    # getObj returns middlepoint or objpos, but no dist atm

    getObject(bot, INR, OTR, robot_pos, C)

    key = C.view(verbose>0, 'results')
    print('key pressed: ', chr(key)) #use this for basic interaction, e.g. aborting the program
    if chr(key)=='q':
        exit()