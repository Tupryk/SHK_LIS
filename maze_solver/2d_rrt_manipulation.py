import time
import numpy as np
import robotic as ry

C = ry.Config()

C.addFrame("base").setPosition([0, 0, .7])

C.addFrame("ego", "base") \
    .setJoint(ry.JT.transXYPhi, [-1.,1.,-1.,1.,-3.,3.]) \
    .setRelativePosition([.0, .0, .0]) \
    .setShape(ry.ST.ssBox, size=[.1, .05, .05, .002]) \
    .setColor([0, 1., 1.]) \
    .setContact(1)

def generate_maze_block(dimensions: list, position: list, frame_name: str):
    C.addFrame(frame_name) \
        .setPosition(position) \
        .setShape(ry.ST.ssBox, size=[*dimensions, .002]) \
        .setColor([1, .5, 0]) \
        .setContact(1)

def generate_maze(size: float=.8, wall_width: float=.06, wall_height: float=.06, random_blocks: int=5, z_pos: float=.7):

    generate_maze_block([wall_width, size, wall_height], [(size+wall_width)*.5, 0, z_pos], "maze_outer_wall_0")
    generate_maze_block([size, wall_width, wall_height], [0, (size+wall_width)*.5, z_pos], "maze_outer_wall_1")
    generate_maze_block([wall_width, size, wall_height], [(size+wall_width)*-.5, 0, z_pos], "maze_outer_wall_2")
    generate_maze_block([size, wall_width, wall_height], [0, (size+wall_width)*-.5, z_pos], "maze_outer_wall_3")

    for i in range(random_blocks):
        w = np.random.choice([.05, .1])
        h = np.random.choice([.05, .1])
        generate_maze_block([w, h, wall_height], [np.random.random()-.5, np.random.random()-.5, z_pos], f"maze_random_wall_{i}")

q0 = [.3, 0, 0]
qT = [-.3, 0, 0]
C.setJointState(qT)
C.addFrame("goal") \
    .setShape(ry.ST.marker, size=[.02]) \
    .setPosition([-.45, 0., .1]) \
    .setColor([1, 0, 1])

# find a cool feasible maze
while True:
    generate_maze()
    rrt = ry.PathFinder()
    rrt.setProblem(C, [q0], [qT])

    ret = rrt.solve()
    print(type(ret))
    print(ret)
    rrt_path = ret.x
    if ret.feasible:
        break

# display the path
for q in rrt_path:
    C.setJointState(q)
    C.view()
    time.sleep(.2)

print("Done!")
C.view(True)

C = ry.Config()
C.addFile(ry.raiPath("scenarios/pandaSingle.g"))
generate_maze()

bot = ry.BotOp(C, False)
bot.home(C)

# Initial positioning
komo = ry.KOMO()
komo.setConfig(C, False)
komo.setTiming(1, 20, 1., 2)

komo.addControlObjective([], 1, 1)
komo.addControlObjective([], 2, 1)

komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
komo.addObjective([1], ry.FS.qItself, [], ry.OT.eq, [10.], [], 1)
komo.addObjective([1], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [0, 0, 1e1], [0, 0, .7])
komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1, 1, 0], [rrt_path[0][0], rrt_path[0][1], 0])

ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
if not ret.feasible:
    print("Error while following path")

bot.moveAutoTimed(komo.getPath())
while bot.getTimeToEnd() > 0:
    bot.sync(C)

# Moving through maze
for i in range(1, len(rrt_path)):
    komo = ry.KOMO()
    komo.setConfig(C, False)
    komo.setTiming(1, 20, 1., 2)

    komo.addControlObjective([], 1, 1)
    komo.addControlObjective([], 2, 1)

    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
    komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [0, 0, 1e1], [0, 0, .7])
    komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1, 1, 0], [rrt_path[i][0], rrt_path[i][1], 0])

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
    if not ret.feasible:
        print("Error while following path")

    bot.moveAutoTimed(komo.getPath())
    while bot.getTimeToEnd() > 0:
        bot.sync(C)
