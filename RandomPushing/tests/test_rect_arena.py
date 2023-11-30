from robotic import ry
import json
import numpy as np
from config import setup_config, startup_robot
from random_paths import compute_motion, run_waypoints_one_by_one
from visual import getObject, lookAtObj
from arena import RectangularArena


WAYPOINTS = 6
#INITIAL_OBJ_POS = [-.5, 0, .69]
INITIAL_OBJ_POS = [-.4, 0, .69]

GRIPPER_WIDTH = .0  #relative gripper width (0 means fully closed, .5 means half open, 1 fully open)
DEBUG = False
OBJ_HEIGHT = .08
RECT_WIDTH=.89
RECT_HEIGHT=.59
INR = .21
OTR = None
ITERATIONS = 10000 #number of rand. pushing iterations

ON_REAL = False
USE_RANSAC = False

robot_pos = np.array([-.03, -.24, .651])
RECT_ARENA_MIDDLEP=np.array([-.23, -.16, .651])
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
    if bot.getKeyPressed()==ord('q'):
        exit()
    #Arena = CircularArena(C=C, middleP=robot_pos, innerR=INR, outerR=OTR)
    # -------- rectArena testing -----
    Arena = RectangularArena(C=C, middleP=RECT_ARENA_MIDDLEP, height=RECT_HEIGHT,  width=RECT_WIDTH, innerR=INR, middlePCirc=robot_pos)
    Arena.plotArena()

    lookAtObj(bot, C, np.array(obj_pos))
    # getObj returns middlepoint or objpos, but no dist atm
    obj_pos  = getObject(bot, C, RECT_ARENA_MIDDLEP, INR, OTR, use_ransac=USE_RANSAC, width=RECT_WIDTH, height=RECT_HEIGHT) 
    if obj_pos:
        dist = np.linalg.norm(C.getFrame("camera").getPosition()-obj_pos)
        if dist != None: 

            for i in range(ITERATIONS):
                bot.sync(C, .1)
                #if bot.getKeyPressed()==ord('q'):
                    #TODO bring home safely
                    #break
                if not obj_pos:
                    print("Can't find object!")
                    break

                bot.home(C)

                #-- compute a motion (debug this inside the method)
                way_start, way_end, _, _, success = Arena.generate_waypoints(C, obj_pos, obj_width=.3, waypoints=WAYPOINTS)

                if not success: break
                path, feasible = compute_motion(C, WAYPOINTS, np.array(way_start) - np.array(way_end), verbose)
                print('returned path shape: ', type(path), path.shape)

                #-- now, if we're happy with the motion, send it to the robot
                if feasible:
                    d = {}
                    d["way_pos"] = {}
                    d["obj_pos"] = {}
                    d["obj_pos"]["start"] = [i for i in obj_pos]
                    d["way_pos"]["start"] = [i for i in way_start]

                    # send the path by individually sending waypoints 
                    run_waypoints_one_by_one(bot, path, True, C)

                else:
                    non_f += 1
                    continue
                        
                lookAtObj(bot, C, np.array(obj_pos))
                obj_pos = getObject(bot, C, RECT_ARENA_MIDDLEP, inner_rad=INR, outer_rad=OTR, use_ransac=USE_RANSAC, width=RECT_WIDTH, height=RECT_HEIGHT)
                
                d["way_pos"]["end"] = [i for i in way_end]

                d["obj_pos"]["end"] = [i for i in obj_pos]

                data.append(d)
                """if chr(key)=='q':
                    del bot
                    del C
                    exit()"""
        
        bot.home(C)
        with open('data.json', 'w') as f:
            json.dump(data, f)

        print("Non feasable paths: ", non_f)
        del bot
        del C
