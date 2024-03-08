import robotic as ry
import manipulation as manip
import numpy as np
import random
import robot_execution as robex

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

midpoint = [-0.105, 0.4, 0.745]
# C.addFrame("obstacle") \
#     .setPosition(midpoint) \
#     .setShape(ry.ST.ssBox, size=[0.09, 0.45, 0.23, 0.001]) \
#     .setColor([0, 0, 1]) \
#     .setContact(1) \
#     .setMass(.1)

for i in range(3):
    box_name = 'box{}'.format(i + 1)

    color = [0, 0, 0]
    color[i] = 1
    position_val1 = 0.1 * (i - 5)
    C.addFrame(box_name) \
        .setPosition([position_val1, 0.05, 0.705]) \
        .setShape(ry.ST.ssBox, size=[0.04, 0.04, 0.12, 0.001]) \
        .setColor(color) \
        .setContact(1) \
        .setMass(.1)

#C.delFrame("panda_collCameraWrist")

# for convenience, a few definitions:
gripper = "l_gripper"
palm = "l_palm"
box = "box1"
table = "table"

C.view()



def build_house(housePosition):
    paths = []

    qStart = C.getJointState()

    box = "box1"
    possible = False
    for graspDirection in ["x", "y", "z"]:
        for placeDirection in ["z", "zNeg"]:
            M = manip.ManipulationModelling(C, helpers=[gripper])
            M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
            M.grasp_box(1., gripper, box, palm, graspDirection)
            M.place_box(2., box, table, palm, placeDirection)
            M.target_relative_xy_position(2., box, table, housePosition)
            M.solve()
            if not M.feasible:
                continue

            # Define the in between joint states when going for the grasp
            M1 = M.sub_motion(0)
            M1.no_collision([.3,.7], palm, box, margin=.05)
            M1.retract([.0, .2], gripper)
            M1.approach([.8, 1.], gripper)
            path1 = M1.solve()
            if not M1.feasible:
                continue

            # Define the in between joint states when placing the object
            M2 = M.sub_motion(1)
            M2.no_collision([], table, box)
            M2.no_collision([.2, .8], table, box, .04)
            path2 = M2.solve()
            if not M2.feasible:
                continue
            possible = True
            break
        if possible:
            break

    if not possible:
        return False
    
    paths.append(path1)
    paths.append(path2)

    C.setJointState(path2[-1])
    X = M.komo.getFrameState(1)
    C.setFrameState(X)
    C.view(True)
    box = "box2"
    possible = False
    for graspDirection in ["x", "y", "z"]:
        for placeDirection in ["z", "zNeg"]:
            M = manip.ManipulationModelling(C, helpers=[gripper])
            M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
            M.grasp_box(1., gripper, box, palm, graspDirection)
            M.place_box(2., box, table, palm, placeDirection)
            M.komo.addObjective([2.], ry.FS.negDistance, ["box2", "box1"], ry.OT.eq, [1e1], [-.08])
            M.solve(verbose=4)
            if not M.feasible:
                continue

            # Define the in between joint states when going for the grasp
            M3 = M.sub_motion(0)
            M3.no_collision([.3,.7], palm, box, margin=.05)
            M3.retract([.0, .2], gripper)
            M3.approach([.8, 1.], gripper)
            path1 = M3.solve()
            if not M3.feasible:
                continue

            # Define the in between joint states when placing the object
            M4 = M.sub_motion(1)
            M4.no_collision([], table, box)
            M4.no_collision([.2, .8], table, box, .04)
            path2 = M4.solve()
            if not M4.feasible:
                continue
            possible = True
            break
        if possible:
            break

    if not possible:
        return False
    
    C.setJointState(qStart)
    
    paths.append(path1)
    paths.append(path2)

    M1.play(C, 1.)
    C.attach(gripper, "box1")
    M2.play(C, 1.)
    C.attach(table, "box1")
    M3.play(C, 1.)
    C.attach(gripper, "box2")
    M4.play(C, 1.)
    C.attach(table, "box2")
    return True

attempt_count = 100
for l in range(attempt_count):
    housePosition = [midpoint[0] + np.random.random()*.4 -.2, midpoint[1] + np.random.random()*.4 -.2]
    if build_house(housePosition):
        break
