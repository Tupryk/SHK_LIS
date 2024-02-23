import time
import numpy as np
import robotic as ry
from typing import List


def generate_maze_block(C: ry.Config, dimensions: list, position: list, frame_name: str):
    #position = np.array(position) + C.getFrame("maze").getPosition()
    C.addFrame(frame_name, "maze") \
        .setRelativePosition(position) \
        .setShape(ry.ST.ssBox, size=[*dimensions, .001]) \
        .setColor([1, .5, 0]) \
        .setContact(1)

def generate_maze(C: ry.Config, position: List[float]=[.0, .0, .0]):

    maze_size: float=.6
    wall_width: float=.04
    wall_height: float=.02

    C.addFrame("maze").setPosition(position)

    generate_maze_block(C, [.01, maze_size, .055], [(maze_size+.01)*.5, 0, 0], "maze_outer_wall_0")
    #generate_maze_block(C, [maze_size, .01, .055], [0, (maze_size+.01)*.5, 0], "maze_outer_wall_1")
    #generate_maze_block(C, [.01, maze_size, .055], [(maze_size+.01)*-.5, 0, 0], "maze_outer_wall_2")
    #generate_maze_block(C, [maze_size, .01, .055], [0, (maze_size+.01)*-.5, 0], "maze_outer_wall_3")
    #generate_maze_block(C, [maze_size, maze_size, wall_height], [.0, .0, -(.055+wall_height)*.5], "maze_floor")

    generate_maze_block(C, [wall_width, .2, .055], [0, 0, 0], "maze_inner_wall")
        
    C.addFrame("start", "maze") \
        .setShape(ry.ST.marker, size=[.02]) \
        .setRelativePosition([.3*maze_size, 0., 0]) \
        .setColor([1, 0, 1])

    C.addFrame("goal", "maze") \
        .setShape(ry.ST.marker, size=[.02]) \
        .setRelativePosition([-.3*maze_size, 0., 0]) \
        .setColor([1, 0, 1])
    
def solve_maze_rrt(C: ry.Config, visual: bool=False) -> ry._robotic.SolverReturn:
    """
    Takes a ry.Config with just some obstacles and a start
    and goal marker frames.

    Returns the output of the RRT solver.

    Will generate an XYPhi joint and ssBox frame.
    """
    maze_pos = C.getFrame("maze").getPosition()
    start_pos = C.getFrame("start").getPosition()
    goal_pos = C.getFrame("goal").getPosition()

    C.addFrame("base").setPosition([*maze_pos[:2], start_pos[2]])
    C.addFrame("ego", "base") \
        .setJoint(ry.JT.transXYPhi, [-1., 1., -1., 1., -3., 3.]) \
        .setShape(ry.ST.ssBox, size=[.05, .1, .05, .002]) \
        .setColor([0, 1., 1.]) \
        .setContact(1)

    q0 = [*start_pos[:2], 0]
    qT = [*goal_pos[:2], 0]

    C.setJointState(qT)
    rrt = ry.PathFinder()
    rrt.setProblem(C, [q0], [qT])
    ret = rrt.solve()

    if visual and ret.feasible:
        C.view(True)
        for i, q in enumerate(ret.x):
            C.addFrame(f"marker{i}") \
                .setShape(ry.ST.marker, size=[.04]) \
                .setPosition([q[0], q[1], 0]) \
                .setColor([1, 1, 1])
            C.setJointState(q)
            C.view()
            time.sleep(.1)
        C.view(True)
    
    return ret
