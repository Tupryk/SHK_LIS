import robotic as ry
import numpy as np

def segment_line(point1, point2, point_between):
    return [point1 + (point2 - point1) * 0.5 * (1-np.cos(np.pi * i/(point_between-1))) for i in range(point_between)]

def push_problem(C, delta):
    '''creates a motion problem using "waypoint engineering" approach: define waypoints and motion relative to these'''

    # define a 2 waypoint problem in KOMO   
    komo = ry.KOMO()
    komo.setConfig(C, True)

    stepsBefore1stWaypoint = 3
    T = 6+stepsBefore1stWaypoint

    komo.setTiming(2, 20, 1., 2)

    komo.addControlObjective([], 0, 1e-2)
    komo.addControlObjective([], 1, 1e-1)
    komo.addControlObjective([], 2, 1e0)
    

    """
    a=C.getFrame("way0").getPosition()
    b=C.getFrame("way5").getPosition()
    delta = b-a"""
    delta /= np.linalg.norm(delta)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([0,1], ry.FS.negDistance, ['l_gripper', 'mid_point'], ry.OT.ineq, [1], [-.1])
    komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', f'way{0}'], ry.OT.eq, [1e1])
    komo.addObjective([1,2], ry.FS.positionDiff, ['l_gripper', f'way{0}'], ry.OT.eq, (np.eye(3)-np.outer(delta,delta)))

    komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', f'way{5}'], ry.OT.eq, [1e1])

    #komo.addObjective([1], ry.FS.qItself, [], ry.OT.eq, [1e1], [], 1)   #no motion 
    komo.addObjective([2], ry.FS.qItself, [], ry.OT.eq, [1e1], [], 1)   #no motion 

    komo.addObjective([1, 2], ry.FS.vectorX, ['l_gripper'], ry.OT.eq, delta.reshape(1,3))
    komo.addObjective([1,2], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1], [0,0,1])
    
    #for i in range(6):
    #    komo.addObjective([i+1+stepsBefore1stWaypoint], ry.FS.positionDiff, ['l_gripper', f'way{i}'], ry.OT.eq, [1e1])

    return komo

def compute_motion(C, delta, verbose=0):
    '''solves the pushProblem'''

    #-- define a motion problem
    komo = push_problem(C, delta)
    if verbose:
        print('this is the defined motion problem: ', komo.reportProblem())

    #-- solve it -> compute motion
    ret = ry.NLP_Solver().setProblem(komo.nlp()).setOptions( stopTolerance=1e-2, verbose=verbose).solve()
    if verbose:
        print(ret)
        print('RET TYPE:',type(ret))

    komo.view_close()
    
    return komo.getPath(), ret.feasible

def run_waypoints_one_by_one(bot, path, wait, C):

    # add the path points by appending individual waypoints to the spline buffer (spline will have zero velocity at waypoints)
    # bot.moveTo(path[0], 1., False)
    # while bot.getTimeToEnd() > 0:
    #     bot.sync(C, .1)

    bot.move(path, [4.])
    while bot.getTimeToEnd() > 0:
        key =bot.sync(C, .1)
        if chr(key)=='q':
                print("Terminated (random paths)")
                bot.home(C)
                del bot
                del C
                exit()
    if wait:
        # wait for the spling buffer to be done, and sync C just for fun
        while bot.getTimeToEnd()>0:
            key = bot.sync(C, .1)
            # print(chr(key))
            if chr(key)=='q':
                print("Terminated (random paths)")
                bot.home(C)
                del bot
                del C
                exit()
            