import time
import robotic as ry


ry.params_add({'rrt/stepsize':.05})

###################### This works ######################
C = ry.Config()

C.addFrame("obstacle") \
    .setPosition([.0, .0, .69]) \
    .setShape(ry.ST.ssBox, size=[.04, .4, .055, .001]) \
    .setColor([1, .5, 0]) \
    .setContact(1)

C.addFrame("base").setPosition([0, 0, .69])
C.addFrame("ego", "base") \
    .setJoint(ry.JT.transXYPhi, [-1., 1., -1., 1., -3., 3.]) \
    .setShape(ry.ST.ssBox, size=[.05, .1, .05, .002]) \
    .setColor([0, 1., 1.]) \
    .setContact(1)

q0 = [.12, 0, 0]
qT = [-.12, 0, 0]

C.setJointState(qT)
rrt = ry.PathFinder()
rrt.setProblem(C, [q0], [qT])
ret = rrt.solve()

C.view(True)
for q in ret.x:
    C.setJointState(q)
    C.view()
    time.sleep(.1)
C.view(True)


###################### This still works ######################
C = ry.Config()

# We change the y position of the obstacle to be .3
C.addFrame("obstacle") \
    .setPosition([.0, .3, .69]) \
    .setShape(ry.ST.ssBox, size=[.04, .4, .055, .001]) \
    .setColor([1, .5, 0]) \
    .setContact(1)

C.addFrame("base").setPosition([0, 0, .69])
C.addFrame("ego", "base") \
    .setJoint(ry.JT.transXYPhi, [-1., 1., -1., 1., -3., 3.]) \
    .setShape(ry.ST.ssBox, size=[.05, .1, .05, .002]) \
    .setColor([0, 1., 1.]) \
    .setContact(1)

# We also change the y position of the joint end state
q0 = [.12, .3, 0]
qT = [-.12, .3, 0]

C.setJointState(qT)
rrt = ry.PathFinder()
rrt.setProblem(C, [q0], [qT])
ret = rrt.solve()

C.view(True)
for q in ret.x:
    C.setJointState(q)
    C.view()
    time.sleep(.1)
C.view(True)


###################### This does not work ######################
C = ry.Config()

# We add a new random frame
C.addFrame("obstacle0") \
    .setPosition([.5, .3, .69]) \
    .setShape(ry.ST.ssBox, size=[.04, .3, .055, .001]) \
    .setColor([1, .5, 0]) \
    .setContact(1)

C.addFrame("obstacle") \
    .setPosition([.0, .3, .69]) \
    .setShape(ry.ST.ssBox, size=[.04, .4, .055, .001]) \
    .setColor([1, .5, 0]) \
    .setContact(1)

C.addFrame("base").setPosition([0, 0, .69])
C.addFrame("ego", "base") \
    .setJoint(ry.JT.transXYPhi, [-1., 1., -1., 1., -3., 3.]) \
    .setShape(ry.ST.ssBox, size=[.05, .1, .05, .002]) \
    .setColor([0, 1., 1.]) \
    .setContact(1)

q0 = [.12, .3, 0]
qT = [-.12, .3, 0]

C.setJointState(qT)
rrt = ry.PathFinder()
rrt.setProblem(C, [q0], [qT])
ret = rrt.solve()

C.view(True)
for q in ret.x:
    C.setJointState(q)
    C.view()
    time.sleep(.1)
C.view(True)

