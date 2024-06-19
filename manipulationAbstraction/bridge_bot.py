import robotic as ry
import numpy as np
from bridgebuild_env import create_n_boxes
import robot_execution as robex
# this import the local manipulation.py .. to be integrated in robotic..
import manipulation as manip
import random

# create task env
C = ry.Config()
C.addFile(ry.raiPath("scenarios/pandaSingle.g"))
n_boxes = 3
keep_dist_between = ["box1", "box2", "box3"]
C = create_n_boxes(C=C, N=n_boxes, position="fixed", shelf = True)

# remove so collision with wrist camera is ignored
C.delFrame("panda_collCameraWrist")

# for convenience, a few definitions:
qHome = C.getJointState()
gripper = "l_gripper"
palm = "l_palm"
box1 = "box1"
box2 = "box2"
box3 = "box3"
table = "table"
boxSize = C.frame(box1).getSize()

C.view()
bot = robex.Robot(C, on_real=True)
bot.goHome(C)

# Komo constraints
C.setJointState(qHome)
C.view_raise()
boxes_pos = [
    C.frame(box1).getPosition(),
    C.frame(box2).getPosition(),
    C.frame(box3).getPosition(),
]
boxes = [box1, box2, box3]
boxes_all = [box1, box2, box3]
n_success = 0
last_box_memory = []
while n_success <= 2:
    print(f"n_success: {n_success}")
    box = np.random.choice(boxes)
    not_picked_boxes = [i for i in boxes_all if i != box]

    qStart = C.getJointState()
    graspDirection = random.choice(["x", "y"])
    placeDirection = "z"
    info = f"placement {n_success}: grasp {graspDirection} place {placeDirection}"
    print("===", info)

    M = manip.ManipulationModelling(C, info, helpers=[gripper])
    M.setup_pick_and_place_waypoints(
        gripper, box, phases = 3., homing_scale=1e-1, accumulated_collisions=True
    )
    M.grasp_box(1.0, gripper, box, palm, graspDirection)
    print("place direction:", placeDirection)

    if n_success > 1:
        offset_xyz = [0.0, 0.0, 0.00]
        print(f"3rd Block: ({box})")
        dir_vec = (
            C.getFrame(last_box_memory[-2]).getPosition()
            - C.getFrame(last_box_memory[-1]).getPosition()
        )
        pos2 = C.getFrame(last_box_memory[-2]).getPosition()
        pos1 = C.getFrame(last_box_memory[-1]).getPosition()
        rel_pos = (pos1 + pos2) / 2 + np.array([0, 0, 0.06])
        dir_vec /= np.linalg.norm(dir_vec)

        M.komo.addObjective([2.0], ry.FS.position, [box], ry.OT.eq, [1e1], rel_pos)
        M.komo.addObjective([2.0], ry.FS.vectorZ, [box], ry.OT.eq, [1e1], dir_vec)

        print("Grasp Direction:", graspDirection)

    if n_success == 1:
        print(f"2nd Block: ({box})")
        M.place_box(2.0, box, table, palm, placeDirection)
        M.komo.addObjective(
            [2.0],
            ry.FS.negDistance,
            [box, last_box_memory[-1]],
            ry.OT.eq,
            [1e1],
            [-0.04],
        )
        M.komo.addObjective(
            [2.0],
            ry.FS.scalarProductXY,
            [box, last_box_memory[-1]],
            ry.OT.eq,
            [1e1],
            [0],
        )
        M.komo.addObjective(
            [2.0],
            ry.FS.positionRel,
            [box, last_box_memory[-1]],
            ry.OT.eq,
            [1.0],
            [0.0, 0.0, 0.1],
        )
        M.keep_dist_pairwise([], keep_dist_between, margin=0.08)
        #M.keep_distance([], box, last_box_memory[-1], margin = 0.06)


    M.keep_distance([], palm, table)
    # keep distance between boxes when placed defined by margin
    # x, y, z, smooth - table size: array([2.5 , 2.5 , 0.1 , 0.02])
    # place inital block as chosen by np.random.choice at a random location
    if n_success == 0:
        print(f"1st Block: ({box})")
        M.place_box(2.0, box, table, palm, placeDirection)
        initial_pos = [np.random.uniform(0.0, 0.1), np.random.uniform(0.0, 0.1)]

        M.target_relative_xy_position(
            2.0, box, table, initial_pos
        )
        M.keep_dist_pairwise([], keep_dist_between, margin=0.08)
        #M.keep_distance([], box, not_picked_boxes[0], margin = 0.06)
        #M.keep_distance([], box, not_picked_boxes[1], margin = 0.06)

    ways = M.solve()

    if not M.feasible:
        print("True")
        continue

    M2 = M.sub_motion(0)
    M2.keep_distance([0.3, 0.7], palm, box)

    M2.retract([0.0, 0.2], gripper, dist=0.012)
    M2.approach([0.8, 1.0], gripper)
    M2.solve()
    if not M2.feasible:
        continue

    M3 = M.sub_motion(1)
    # manip.ManipulationModelling(C, info)
    # M.setup_point_to_point_motion(ways[0], ways[1])
    M3.keep_distance([], table, box)


    M3.retract([0.0, 0.2], gripper, dist=0.025)
    M3.approach([0.8, 1.0], gripper)

    M3.solve()
    if not M3.ret.feasible:
        continue

    M2.play(C)
    C.attach(gripper, box)
    M3.play(C)
    C.attach(table, box)
    # update memory
    last_box_memory.append(box)
    # remove box that was previously used
    boxes.remove(box)
    # update positions
    boxes_pos = [
        C.frame(box1).getPosition(),
        C.frame(box2).getPosition(),
        C.frame(box3).getPosition(),
    ]
    n_success += 1


del M