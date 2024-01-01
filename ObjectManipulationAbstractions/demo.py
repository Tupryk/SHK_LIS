import robotic as ry

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

C.addFrame('block') \
    .setPosition([-.15,.1,.67]) \
    .setShape(ry.ST.ssBox, size=[.06,.12,.06,.002]) \
    .setColor([1,.5,0]) \
    .setContact(True) \
    .setMass(1e-2)

bot = ry.BotOp(C, False)
bot.home(C)

C.view()

moveObjectTo(bot, C, "block", [.0,.12, .67])

C.view(True)
