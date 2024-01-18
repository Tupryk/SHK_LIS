import numpy as np
import robotic as ry
from typing import Tuple


def moveBlocking(bot: ry.BotOp, C: ry.Config, komo: ry.KOMO, velocity: float, verbose: int=0) -> bool:
    
    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    if verbose: print(ret)
    if verbose > 1 and ret.feasible: komo.view(True)

    if ret.feasible:
        
        bot.moveAutoTimed(komo.getPath(), velocity) # moveAutoTimed prints out two numbers, idk why
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        return True
    
    print("Error while executing robot movement!")
    return False

    
def moveBlockingAndCheckForce(
        bot: ry.BotOp,
        C: ry.Config,
        komo: ry.KOMO,
        velocity: float,
        maxForceAllowed: float=np.nan,
        verbose: int=0) -> Tuple[bool, float]:

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    if verbose: print(ret)
    if verbose > 1 and ret.feasible: komo.view(True)

    if ret.feasible:
        
        bot.moveAutoTimed(komo.getPath(), velocity)
        
        max_force = -np.inf
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)
            y, J = C.eval(ry.FS.position, ['l_gripper'], [[0, 1, 0]])
            F = np.abs(J @ bot.get_tauExternal())
            max_force = F if F > max_force else max_force
            if max_force > maxForceAllowed:
                print("Max force exceeded!")
                break
        
        if verbose: print("Max force enacted: ", max_force)
        
        return True, max_force
    
    print("Error while executing robot movement!")
    return False, 0.
