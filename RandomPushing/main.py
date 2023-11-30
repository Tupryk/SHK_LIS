from robotic import ry
import json
import numpy as np
from config import setup_config, startup_robot
from random_paths import compute_motion, run_waypoints_one_by_one
from visual import getObject, lookAtObj
from arena import *


INITIAL_OBJ_POS = [-.5, 0, .69]
DEBUG = False
OBJ_HEIGHT = .08

ON_REAL = True
NUMBER_ITERATIONS=100
table_width=.89
table_height=.55
table_middle=np.array([-.23, -.16, .651])

innerR= .29
robot_pos = np.array([-.03, -.24, .651])



if __name__ == "__main__":

    #-- load parameters, typically automatically from 'rai.cfg'
    ry.params_print()
    verbose = 0

    #-- define a configuration
    C = setup_config(INITIAL_OBJ_POS, ON_REAL)
    if DEBUG:
        key = C.view(verbose>0, 'happy with the config?')
        print('key pressed: ', chr(key)) #use this for basic interaction, e.g. aborting the program
        if chr(key)=='q':
            exit()

    bot = startup_robot(C, ON_REAL)

    data = []
    obj_pos = INITIAL_OBJ_POS

    # Generate Arena
    arena = RectangularArena(middleP=table_middle, width=table_width, height=table_height, middlePCirc=robot_pos, innerR=innerR)
    arena.plotArena(C)

    # Point towards set initial object position
    lookAtObj(bot, C, np.array(obj_pos))

    # Capture midpoint from point cloud
    obj_pos  = getObject(bot, C, arena)
    
    for i in range(NUMBER_ITERATIONS):

        if not obj_pos: break
        key=bot.sync(C, .1)
        if chr(key) == "q":
            print("Terminated (main loop)")
            break

        #-- compute a motion (debug this inside the method)
        way_start, way_end, pred_point, delta, success = arena.generate_waypoints(obj_pos, obj_width=.3, ry_config=C)

        if not success: break
        path, feasible = compute_motion(C, delta, verbose)
        print('returned path shape: ', type(path), path.shape)

        #-- now, if we're happy with the motion, send it to the robot
        if feasible:
            obj_start = obj_pos

            # Move object
            run_waypoints_one_by_one(bot, path, True, C)

            # Check new object position
            lookAtObj(bot, C, np.array(pred_point))

            obj_pos = getObject(bot, C, arena)
            
            # Store information
            data.append({
                "way_pos": {
                    "start": [i for i in way_start],
                    "end": [i for i in way_end]
                },
                "obj_pos": {
                    "start": [i for i in obj_start],
                    "end": [i for i in obj_pos]
                }
            })

    bot.home(C)
    json.dump(data, open('data/data.json', 'w'))
