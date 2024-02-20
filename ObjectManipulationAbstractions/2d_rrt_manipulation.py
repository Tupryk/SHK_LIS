import time
import numpy as np
import robotic as ry

C = ry.Config()
C.addFrame("base").setPosition([0, 0, .1])

C.addFrame("ego", "base") \
    .setJoint(ry.JT.transXYPhi, [-1.,1.,-1.,1.,-3.,3.]) \
    .setRelativePosition([.0, .0, 0]) \
    .setShape(ry.ST.ssBox, size=[.05, .05, .05, .002]) \
    .setColor([0, 1., 1.]) \
    .setContact(1)

def generate_maze_block(dimensions: list, position: list, frame_name: str):
    C.addFrame(frame_name) \
        .setPosition(position) \
        .setShape(ry.ST.ssBox, size=[*dimensions, .002]) \
        .setColor([1, .5, 0]) \
        .setContact(1)

def generate_maze(size: float=1., wall_width: float=.06, wall_height: float=.06, random_blocks: int=15):

    generate_maze_block([wall_width, size, wall_height], [(size+wall_width)*.5, 0, .1], "maze_outer_wall_0")
    generate_maze_block([size, wall_width, wall_height], [0, (size+wall_width)*.5, .1], "maze_outer_wall_1")
    generate_maze_block([wall_width, size, wall_height], [(size+wall_width)*-.5, 0, .1], "maze_outer_wall_2")
    generate_maze_block([size, wall_width, wall_height], [0, (size+wall_width)*-.5, .1], "maze_outer_wall_3")

    for i in range(random_blocks):
        w = np.random.choice([.05, .1])
        h = np.random.choice([.05, .1])
        generate_maze_block([w, h, wall_height], [np.random.random()-.5, np.random.random()-.5, .1], f"maze_random_wall_{i}")

q0 = [.45, 0, 0]
qT = [-.45, 0, 0]
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
    print(ret)
    path = ret.x
    if ret.feasible:
        break

# display the path
for q in path:
    C.setJointState(q)
    C.view()
    time.sleep(.2)

print("Done!")
C.view(True)
