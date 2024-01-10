import robotic as ry


def moveLocking(bot: ry.BotOp, C: ry.Config,komo: ry.KOMO, timeToTravel: float, verbose: int=0) -> bool:

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    if verbose: print(ret)
    if verbose > 1 and ret.feasible: komo.view(True)

    if ret.feasible:

        bot.move(komo.getPath(), [timeToTravel])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        return True
    
    print("Error while executing robot movement!")
    return False
    