import robotic as ry
import numpy as np
import time

ON_REAL = True

ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))


th = .67    #table height
block1x, block1y = -.3, .04
block2x, block2y = block1x+.1, block1y
block3x, block3y = block1x+.2, block1y

dropoffHeight = .1  #.12 funktionierte bei bestimmter anordnung einmal

#adding block objects
C.addFrame('block1') \
    .setPosition([block1x, block1y, th]) \
    .setShape(ry.ST.ssBox, size=[.06,.12,.06,.002]) \
    .setColor([1,.5,0]) \
    .setContact(True) \
    .setMass(1e-2)

C.addFrame('block2') \
    .setPosition([block2x,block3y,th]) \
    .setShape(ry.ST.ssBox, size=[.06,.12,.06,.002]) \
    .setColor([1,.5,0]) \
    .setContact(True) \
    .setMass(1e-2)

C.addFrame('block3') \
    .setPosition([block3x,block3y,th]) \
    .setShape(ry.ST.ssBox, size=[.06,.12,.06,.002]) \
    .setColor([1,.5,0]) \
    .setContact(True) \
    .setMass(1e-2) 

C.addFrame('block1way1'). setShape(ry.ST.marker, [.1]) .setPosition([block1x, block1y+.02, th+dropoffHeight])
C.addFrame('block1way2'). setShape(ry.ST.marker, [.1]) .setPosition([block1x, block1y+.02, th+dropoffHeight])
C.addFrame('block1way3'). setShape(ry.ST.marker, [.1]) .setPosition([block1x, block1y+.02, th+dropoffHeight-.1])
C.addFrame('block1way4'). setShape(ry.ST.marker, [.1]) .setPosition([block1x-.15, block1y+.02, th+dropoffHeight-.1])

C.addFrame('block2way1'). setShape(ry.ST.marker, [.1]) .setPosition([block2x, block2y+.02, th+dropoffHeight])
C.addFrame('block2way2'). setShape(ry.ST.marker, [.1]) .setPosition([block2x-.35, block2y+.02, th+dropoffHeight])
C.addFrame('block2way3'). setShape(ry.ST.marker, [.1]) .setPosition([block2x-.35, block2y+.02, th+dropoffHeight-.1])
C.addFrame('block2way4'). setShape(ry.ST.marker, [.1]) .setPosition([block2x-.15, block2y+.02, th+dropoffHeight-.1])

C.addFrame('block3way1'). setShape(ry.ST.marker, [.1]) .setPosition([block3x, block3y, th+dropoffHeight])
C.addFrame('block3way2'). setShape(ry.ST.marker, [.1]) .setPosition([block3x-.35,block3y, th+dropoffHeight+.03])
C.addFrame('block3way3'). setShape(ry.ST.marker, [.1]) .setPosition([block3x-.35,block3y, th+dropoffHeight-.02])
C.addFrame('block3way4'). setShape(ry.ST.marker, [.1]) .setPosition([block3x-.15,block3y, th+dropoffHeight-.09])



def syncAndExit(C, sync_time=.1, homing = True):
    key = bot.sync(C, .1)
    if chr(key) == "q":
        if(homing==True):
            bot.home(C)
        exit()

def openGripper(C, width=.075, speed=.2):
    bot.gripperMove(ry._left, width=width, speed=speed)

    while not bot.gripperDone(ry._left):
        syncAndExit


def grabBlock(C, bot, block):
    qHome = C.getJointState()

    komo = ry.KOMO(C, 1, 1, 0, False)
    komo.addObjective(
        times=[], 
        feature=ry.FS.jointState, 
        frames=[],
        type=ry.OT.sos, 
        scale=[1e-1], 
        target=qHome
    )
    komo.addObjective([], ry.FS.positionDiff, ['l_gripper', block], ry.OT.eq, [1e1])

    ret = ry.NLP_Solver(komo.nlp(), verbose=4) .solve()
    print(ret)

    q = komo.getPath()
    del komo #also closes komo view

    C.setJointState(q[0])

    komo = ry.KOMO(C, 1,1,0, True)
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)    # sum of collision penetrations; when negative/zero, nothing is colliding
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)            # sum of joint limit penetrations; when negative/zero, all joint limits are ok


    komo.addObjective([], ry.FS.positionDiff, ['l_gripper', block], ry.OT.eq, [1e1])
    komo.addObjective([], ry.FS.scalarProductXY, ['l_gripper', block], ry.OT.eq, [1e1], [0])
    komo.addObjective([], ry.FS.scalarProductXZ, ['l_gripper', block], ry.OT.eq, [1e1], [0])
    komo.addObjective([], ry.FS.distance, ['l_palm', block], ry.OT.ineq, [1e1])

    ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()
    print(ret)
    if ret.feasible:
        print('--- FEASIBLE ---')
    else:
        print('---  !!!!!!!!!!! NOT FEASIBLE !!!!!!!!!!! ---')

    q = komo.getPath()
    C.setJointState(q[0])

    bot.move(q, [5])


    while bot.getTimeToEnd()>0:
        syncAndExit(C)

    bot.gripperClose(ry._left, width=.03, speed=.1)

    while not bot.gripperDone(ry._left):
        syncAndExit(C)

def maneuverTo(C, bot, block, position="orthogonal"):

    #TODO fett änder  maybe LaTeX document mit allen KOMO sachen erweitern  vllt etwas wie x coordinate > -.4?

    #Get to waypint then after
    komo = ry.KOMO(C, phases=4, slicesPerPhase=1, kOrder=1, enableCollisions=True)
    komo.addControlObjective([], 0, 1e-1)         # recherchieren  position?     
    komo.addControlObjective([], 1, 1e0)          # recherchieren  geschwindigkeit?

    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)        # recherchieren
    komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', f'{block}way1'], ry.OT.eq, [1e1])
    komo.addObjective([2], ry.FS.positionDiff, ['l_gripper', f'{block}way2'], ry.OT.eq, [1e1])
    if(position=="orthogonal"):
        komo.addObjective([2], ry.FS.scalarProductYZ, ['l_gripper', f'{block}way2'], ry.OT.eq, [1e1], [1])
        komo.addObjective([2], ry.FS.scalarProductXX, ['l_gripper', f'{block}way2'], ry.OT.eq, [1e1], [0])

    elif(position=="parallel"):
        komo.addObjective([2], ry.FS.scalarProductXX, ['l_gripper', f'{block}way2'], ry.OT.eq, [1e1], [0])
        komo.addObjective([2], ry.FS.scalarProductYY, ['l_gripper', f'{block}way2'], ry.OT.eq, [1e1], [0])         


    komo.addObjective([3], ry.FS.positionDiff, ['l_gripper', f'{block}way3'], ry.OT.eq, [1e1])
    komo.addObjective([4], ry.FS.positionRel, [f'{block}way3', "cameraWrist"], ry.OT.eq, [1.], [.0, .0, .2])


    ret = ry.NLP_Solver(komo.nlp(), verbose=0 ) .solve()
    print(ret)
    if ret.feasible:
        print('--- FEASIBLE ---')
    else:
        print('---  !!!!!!!!!!! NOT FEASIBLE !!!!!!!!!!! ---')
    q = komo.getPath()
    print(q)
    C.setJointState(q[0])

    #komo.view(True)
    bot.move(q[:-1], [1,3,4.5])  # save last pathstep for after loslassing

    while bot.getTimeToEnd()>0:
        syncAndExit(C)

    openGripper(C)

    bot.move(np.asarray([C.getJointState(), q[-1]]), [1])  # save last pathstep for after loslassing

    while bot.getTimeToEnd()>0:
        syncAndExit(C)



#startup robot
bot = ry.BotOp(C, ON_REAL)
bot.home(C)

bot.gripperMove(ry._left, 0.07, .1)
while not bot.gripperDone(ry._left):
    syncAndExit(C)


grabBlock(C, bot, "block1")
maneuverTo(C, bot, "block1")
bot.home(C)


grabBlock(C, bot, "block2")
maneuverTo(C, bot, "block2")
bot.home(C)

grabBlock(C, bot, "block3")
maneuverTo(C, bot, "block3", "parallel")
bot.home(C)