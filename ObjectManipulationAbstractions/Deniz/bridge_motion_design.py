import robotic as ry
import numpy as np
import random
from bridgebuild_env import create_n_boxes

# this import the local manipulation.py .. to be integrated in robotic..
import manipulation as manip
import robot_execution as robex


C = ry.Config()
C.addFile(ry.raiPath("scenarios/pandaSingle.g"))
n_boxes = 3
keep_dist_between = ["box1", "box2", "box3"]
C = create_n_boxes(C=C, N=n_boxes, position="fixed")

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

C.setJointState(qHome)
C.view_raise()
boxes_pos = [
    C.frame(box1).getPosition(),
    C.frame(box2).getPosition(),
    C.frame(box3).getPosition(),
]
boxes = [box1, box2, box3]
last_box_memory = []
n_success = 0
while n_success <= 2:
    box = np.random.choice(boxes)
    qStart = C.getJointState()
    graspDirection = random.choice(["x", "y"])  # random.choice(['y', 'z']) #'x' not possible: box too large
    placeDirection = "z"  # random.choice(['x', 'y', 'z', 'xNeg', 'yNeg', 'zNeg'])
    info = f"placement {n_success}: grasp {graspDirection} place {placeDirection}"
    print("===", info)

    M = manip.ManipulationModelling(C, info, helpers=[gripper])
    M.setup_pick_and_place_waypoints(
        gripper, box, homing_scale=1.5e-1, accumulated_collisions=True
    )
    M.grasp_box(1.0, gripper, box, palm, graspDirection)
    print("place direction:", placeDirection)

    if n_success > 1:

        print(f"3rd Block: ({box})")

        dir_vec = (
            C.getFrame(last_box_memory[-2]).getPosition()
            - C.getFrame(last_box_memory[-1]).getPosition()
        )
        pos2 = C.getFrame(last_box_memory[-2]).getPosition()
        pos1 = C.getFrame(last_box_memory[-1]).getPosition()
        rel_pos = (pos1 + pos2) / 2 + np.array([0, 0, 0.080])
        dir_vec /= np.linalg.norm(dir_vec)

        M.komo.addObjective([2.0], ry.FS.position, [box], ry.OT.eq, [1e1], rel_pos)
        M.komo.addObjective([2.0], ry.FS.vectorZ, [box], ry.OT.eq, [1e1], dir_vec)
        # M.keep_dist_pairwise([], keep_dist_between, margin=0.08)

        print("Grasp Direction:", graspDirection)
        # M.komo.addObjective([2.], ry.FS.vectorZ, [gripper], ry.OT.eq, [1e1], [0, 0, 1])

        if graspDirection == "x":
            M.komo.addObjective(
                [2.0], ry.FS.scalarProductXZ, [box, table], ry.OT.eq, [1e1], [0.0]
            )
        elif graspDirection == "y":
            M.komo.addObjective(
                [2.0], ry.FS.scalarProductYZ, [box, table], ry.OT.eq, [1e1], [0.0]
            )
        M.keep_distance([], last_box_memory[-1], gripper)
        M.keep_distance([], last_box_memory[-2], gripper)
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

        M.keep_distance([], last_box_memory[-1], gripper)

    #M.keep_distance([], palm, table)
    # keep distance between boxes when placed defined by margin
    # x, y, z, smooth - table size: array([2.5 , 2.5 , 0.1 , 0.02])
    # place inital block as chosen by np.random.choice at a random location
    if n_success == 0:
        print(f"1st Block: ({box})")
        M.place_box(2.0, box, table, palm, placeDirection)
        initial_pos = [np.random.uniform(0.0, 0.1), np.random.uniform(0.05, 0.25)]

        M.target_relative_xy_position(2.0, box, table, initial_pos)

    
    ways = M.solve(verbose=2)

    if not M.feasible:
        print(
            f"Failed placing block number {n_success+1} with grasp direction {graspDirection} at M :("
        )
        continue
    print("Submotion 0")

    M2 = M.sub_motion(0)

    #M2.keep_distance([0.3, 0.7], palm, box)

    M2.retract([0.0, 0.2], gripper, dist=0.13)
    M2.approach([0.8, 1.0], gripper)
    M2.solve(verbose=2)
    if not M2.feasible:
        print(
            f"Failed placing block number {n_success+1} with grasp direction {graspDirection} at M2 :("
        )
        continue

    M3 = M.sub_motion(1)

    #M3.keep_distance([], table, box)
    
    if n_success > 1:
        M3.retract([0.0, 0.2], gripper, dist=0.13)
        M3.approach([0.8, 1.0], gripper)


    M3.solve(verbose=2)
    if not M3.ret.feasible:
        print(
            f"Failed placing block number {n_success+1} with grasp direction {graspDirection} at M3 :("
        )
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