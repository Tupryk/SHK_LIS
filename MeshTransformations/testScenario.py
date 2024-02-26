import robotic as ry

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(True)
bot = ry.BotOp(C, useRealRobot=False)

qHome = bot.get_qHome()
q0 = qHome.copy()
q1 = q0.copy()


for i in range(100000):
    q1[1] = q1[1] + (((i%2)*2)-1)*.05
    bot.moveTo(q1)

    while bot.getTimeToEnd()>0:
        bot.sync(C, .1)