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
