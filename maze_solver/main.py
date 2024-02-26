import robotic as ry
import manipulation as manip
import robot_execution as robex
from komo_path import solve_path_motion
from maze_utils import generate_maze, solve_maze_rrt

"""
First we need to find a path through the maze. To do
this we create an empty config and add an XYPhi joint
to which we will attach a block representing the box
and robot gripper.

Once this in done we can define an end joint state at
the end of the maze. Finally we just call the RRT solver
and get the path we need to follow in order to solve
the puzzle.
"""

ry.params_add({'rrt/stepsize':.05})

C = ry.Config()
generate_maze(C, [.0, .3, .69])
ret = solve_maze_rrt(C, True)

if not ret.feasible:
    print("The RRT solver was unable to find a feasible path.")
    exit()

"""
Now that we have a path for the robot to follow we can
use komo to move the robot arm through the maze.
"""

# Define the new config
C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

generate_maze(C, [.0, .3, .69])

bpos = C.getFrame("start").getPosition()
C.addFrame("box") \
    .setPosition(bpos) \
    .setShape(ry.ST.ssBox, size=[.03, .03, .03, .001]) \
    .setColor([0., 0., 1.]) \
    .setContact(1) \
    .setMass(1.)

C.view()

# Grab the box and follow the RRT path
man = manip.ManipulationModelling(C)
man.setup_inverse_kinematics()
man.grasp_box(1., "l_gripper", "box", "l_palm", "y")
pose = man.solve()

if man.feasible:
    robot = robex.Robot(C)
    robot.execute_path_blocking(C, pose)
    robot.grasp(C)

    man = manip.ManipulationModelling(C)
    man.follow_path_on_plane(ret.x)
    path = man.solve()
    
    try:
        robot.execute_path_blocking(C, path)
    except:
        print("Path is not feasible!")

C.view(True)
