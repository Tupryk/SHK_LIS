import robotic as ry

C = ry.Config()
ry.params_print()
#C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))

bot = ry.BotOp(C, True)
#bot.home(C)

C.view(True)
