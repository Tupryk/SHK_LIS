from robotic import ry

#gripper width floatvalue [0, 0.0790791]
def homing(gripper_width=.01):

    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

    bot = ry.BotOp(C, True)
    bot.home(C)

    bot.gripperMove(ry._left, gripper_width, .3)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    del bot
    del C
    exit()

if __name__ == "__main__":
    homing(.9)
