import robotic as ry

def setup_config(obj_pos=[-.50, .1, .69], on_real=False, debug=False):
    '''creates a config to work with'''

    C = ry.Config()
    #ry.params_add({'botsim/engine': 'kinematic'})
    C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

    # f = C.addFrame("camera", "l_gripper")
    # f.setShape(ry.ST.camera, [.1])
    # f.addAttributes({'focalLength':0.895, 'width':640., 'height':360.})

    if debug:
        C.addFrame("z-limit") \
            .setShape(ry.ST.ssBox, size=[2., 2., .1, .005]) \
            .setPosition([0., 0., 1]) \
            .setColor([1., 0., 0., .5])
    
    if not on_real:
        C.addFrame('obj') \
            .setPosition(obj_pos) \
            .setShape(ry.ST.cylinder, size=[.08, .06]) \
            .setColor([1, .5, 0]) .setMass(.1) .setContact(True)
    
    C.addFrame('predicted_obj') \
        .setShape(ry.ST.marker, size=[.1]) \
        .setPosition(obj_pos) \
        .setColor([1, 0, 0])

    for i in range(6):
        C.addFrame(f'way{i}') \
            .setShape(ry.ST.sphere, size=[.01]) \
            .setPosition([0, .0, .0]) \
            .setColor([0, 0, 1])
        
    for i in range(32):
        C.addFrame(f'view_point_{i}') \
            .setPosition([0, 0, 0]) \
            .setShape(ry.ST.sphere, size=[.02]) \
            .setColor([1, 1, 0])
    
    return C

def startup_robot(C, on_real):
    '''start the robot (or simulation)'''

    bot = ry.BotOp(C, on_real)
    bot.home(C)

    bot.gripperMove(ry._left, .01, .4)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    return bot
