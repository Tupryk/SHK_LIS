
import robotic as ry
import numpy as np
import random
import time

import manipulation as manip

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

bot = ry.BotOp(C, useRealRobot=False)
#bot.gripperMove(ry._left, .0)

th = .67    #table height


dropoffHeight = .1  #.12 funktionierte bei bestimmter anordnung einmal

#adding block objects
C.addFrame('block1') \
    .setPosition([.25, .12, th]) \
    .setShape(ry.ST.ssBox, size=[.12, .04, .04, .002]) \
    .setColor([1,.5,0]) \
    .setContact(True) \
    .setMass(1e-2)

C.addFrame('block2') \
    .setPosition([.25, .24, th]) \
    .setShape(ry.ST.ssBox, size=[.12, .04, .04, .002]) \
    .setColor([1,.5,0]) \
    .setContact(True) \
    .setMass(1e-2)

C.addFrame('block3') \
    .setPosition([.25, .36, th]) \
    .setShape(ry.ST.ssBox, size=[.12, .04, .04, .002]) \
    .setColor([1,.5,0]) \
    .setContact(True) \
    .setMass(1e-2) 


# for convenience, a few definitions:
qHome = C.getJointState()
gripper = "l_gripper"
palm = "l_palm"
box = "block1"
table = "table"
boxSize = C.frame(box).getSize()

C.view(True)

C.setJointState(qHome)
   
# setup the manipulation problem
man = manip.ManipulationModelling(C)
man.setup_pick_and_place_waypoints(gripper, "block1")
man.grasp_top_box(1., gripper, box, "yz")
man.place_box(2., box, table, palm, "x")
man.target_relative_xy_position(2., box, table, [-.15, .4])

# solve it
pose = man.solve()

# check feasibility and display
if man.ret.feasible:
    C.setJointState(pose[0])
    C.view(True, f'grasp_top_box with orientation "yz"\nret: {man.ret}')
    q = pose
    print(type(q), len(q))
    print(q[0],q[-1])

    C.setJointState(pose[1])
    C.view(True, f'grasp_top_box with orientation "yz"\nret: {man.ret}')
    # bot.moveTo(q[0])

    # while True:
    #     bot.sync(C, .1)
else:
    print(' -- infeasible')

