import robotic as ry

def run_waypoints_one_by_one(path, bot, C, wait=True):

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

def moveLocking(bot: ry.BotOp, C: ry.Config,komo: ry.KOMO, timeToTravel: float, verbose: int=0) -> bool:

    ret = ry.NLP_Solver().setProblem(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=verbose).solve()
    if verbose: print(ret)
    if verbose > 1 and ret.feasible: komo.view(True)

    if ret.feasible:

        bot.move(komo.getPath(), [timeToTravel])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        return True
    
    print("Error while executing robot movement!")
    return False
    