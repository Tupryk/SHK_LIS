import robotic as ry
import manipulation as manip
import robot_execution as robex


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

C.addFrame('box') \
    .setPosition([-.25,.1,1.]) \
    .setShape(ry.ST.ssBox, size=[.06,.06,.06,.005]) \
    .setColor([1,.5,0]) \
    .setContact(1)


# for convenience, a few definitions:
qHome = C.getJointState()
boxSize = C.frame("box").getSize()

C.view(True)

man = manip.ManipulationModelling(C)
man.setup_inverse_kinematics()
man.grasp_box(1., "l_gripper", "box", "l_palm", "z", margin=.02)
pose = man.solve()

if man.feasible:
    C.setJointState(pose[0])
    C.view(True)
    C.setJointState(qHome)

    robot = robex.Robot(C)
    robot.execute_path_blocking(C, pose)

while True:
    robot.bot.sync(C, .1)

C.view(True)
