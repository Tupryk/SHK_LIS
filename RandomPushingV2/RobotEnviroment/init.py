import robotic as ry

def setup_config(obj_pos=[-.50, .1, .69], on_real=False):

    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
    
    if not on_real:
        C.addFrame('obj') \
            .setPosition(obj_pos) \
            .setShape(ry.ST.cylinder, size=[.08, .06]) \
            .setColor([1, .5, 0]) .setMass(.1) .setContact(True)
    
    C.addFrame('predicted_obj') \
        .setShape(ry.ST.marker, size=[.1]) \
        .setPosition(obj_pos) \
        .setColor([1, 0, 0])
    
    return C

def startup_robot(C, on_real):

    bot = ry.BotOp(C, on_real)
    bot.home(C)

    bot.gripperMove(ry._left, .01, .4)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    
    return bot
