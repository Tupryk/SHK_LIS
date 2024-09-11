import robotic as ry
import manipulation as manip
import numpy as np
import robot_execution as robex

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

midpoint = [-0.105, 0.2, 0.745]
C.addFrame("obstacle") \
    .setPosition(midpoint) \
    .setShape(ry.ST.marker, size=[0.07]) \
    .setColor([0, 0, 1]) \

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



def return_house_path(housePosition):
    bridge_paths = []

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
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)

    frameStart = M.komo.getFrameState(0)
    X = M.komo.getFrameState(1)
    C.setFrameState(X)
    C.setJointState(path2[-1])

    box = "box2"
    possible = False
    for graspDirection in ["x", "y", "z"]:
        for placeDirection in ["z", "zNeg"]:
            M = manip.ManipulationModelling(C, helpers=[gripper])
            M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
            M.grasp_box(1., gripper, box, palm, graspDirection)
            M.place_box(2., box, table, palm, placeDirection)
            M.komo.addObjective([2.], ry.FS.negDistance, ["box2", "box1"], ry.OT.eq, [1e1], [-.05])
            M.solve()
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
            M4.no_collision([], "box1", palm)
            M4.no_collision([], "box2", palm)
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
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)
    
    X = M.komo.getFrameState(1)
    C.setFrameState(X)
    C.setJointState(path2[-1])

    box = "box3"
    possible = False
    for graspDirection in ["x", "y"]:
        M = manip.ManipulationModelling(C, helpers=[gripper])
        M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
        M.grasp_box(1., gripper, box, palm, grasp_direction="y")
        
        box3_dir = C.getFrame("box2").getPosition() - C.getFrame("box1").getPosition()
        box3_dir /= np.linalg.norm(box3_dir)

        M.komo.addObjective([2.], ry.FS.vectorZ, [box], ry.OT.eq, [1e1], box3_dir)
        M.komo.addObjective([2.], ry.FS.vectorX, [box], ry.OT.eq, [1e1], [0, 0, -1])
        end_pos = housePosition + box3_dir *.05
        end_pos[2] = .69 + .11
        M.komo.addObjective([2.], ry.FS.position, [box], ry.OT.eq, [1e1], end_pos)
        M.solve()
        if not M.feasible:
            continue

        # Define the in between joint states when going for the grasp
        M5 = M.sub_motion(0)
        M5.no_collision([.3, .7], palm, box, margin=.05)
        M5.retract([.0, .2], gripper)
        M5.approach([.8, 1.], gripper)
        path1 = M5.solve()
        if not M5.feasible:
            continue

        # Define the in between joint states when placing the object
        M6 = M.sub_motion(1)
        M6.no_collision([], table, box)
        M6.no_collision([], "box1", palm)
        M6.no_collision([], "box2", palm)
        M6.no_collision([.0, .9], "box1", box)
        M6.no_collision([.0, .9], "box2", box)
        M6.no_collision([.2, .8], table, box, .04)
        M6.approach([.8, 1.], gripper)
        path2 = M6.solve()
        if not M6.feasible:
            continue
        possible = True
        break

    if not possible:
        return False
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)
    
    C.setFrameState(frameStart)
    C.setJointState(qStart)


    C.view()
    # robot = robex.Robot(C, on_real=False)
    # robot.goHome(C)
    # for i in range(3):
    #     box = f"box{i+1}"
    #     robot.execute_path_blocking(C, bridge_paths[i*2])
    #     robot.grasp(C)
    #     C.attach(gripper, box)
        
    #     robot.execute_path_blocking(C, bridge_paths[i*2+1])
    #     C.attach(table, box)
    #     robot.release(C)


    return bridge_paths


def move_blocks(housePosition):
    bridge_paths = []

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
            
            M.komo.addObjective([1., 2.], ry.FS.angularVel, [gripper], ry.OT.sos, [1e1])

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
            M2.no_collision([.2, .8], table, box, .06)
            path2 = M2.solve()
            if not M2.feasible:
                continue
            possible = True
            break
        if possible:
            break

    if not possible:
        return False
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)

    frameStart = M.komo.getFrameState(0)
    X = M.komo.getFrameState(1)
    C.setFrameState(X)
    C.setJointState(path2[-1])

    box = "box2"
    possible = False
    for graspDirection in ["x", "y", "z"]:
        for placeDirection in ["z", "zNeg"]:
            M = manip.ManipulationModelling(C, helpers=[gripper])
            M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
            M.grasp_box(1., gripper, box, palm, graspDirection)
            M.place_box(2., box, table, palm, placeDirection)
            M.komo.addObjective([1., 2.], ry.FS.quaternionDiff, [gripper, gripper], ry.OT.eq, [1e1])
            M.komo.addObjective([2.], ry.FS.negDistance, ["box2", "box1"], ry.OT.eq, [1e1], [-.1])
            M.komo.addObjective([2.], ry.FS.negDistance, ["box2", "box3"], ry.OT.ineq, [1e1], [-.07])

            M.komo.addObjective([1., 2.], ry.FS.angularVel, [gripper], ry.OT.sos, [1e1])

            M.solve()
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
            M4.no_collision([], "box1", palm)
            M4.no_collision([], "box2", palm)
            M4.no_collision([.2, .8], table, box, .12)
            path2 = M4.solve()
            if not M4.feasible:
                continue
            possible = True
            break
        if possible:
            break

    if not possible:
        return False
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)
    
    X = M.komo.getFrameState(1)
    C.setFrameState(X)
    C.setJointState(path2[-1])

    box = "box3"
    possible = False
    for graspDirection in ["x", "y", "z"]:
        for placeDirection in ["z", "zNeg"]:
            M = manip.ManipulationModelling(C, helpers=[gripper])
            M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
            M.grasp_box(1., gripper, box, palm, graspDirection)
            M.place_box(2., box, table, palm, placeDirection)
            M.komo.addObjective([2.], ry.FS.negDistance, ["box3", "box2"], ry.OT.eq, [1e1], [-.1])
            M.komo.addObjective([2.], ry.FS.negDistance, ["box3", "box2"], ry.OT.ineq, [1e1], [-.15])

            M.komo.addObjective([1., 2.], ry.FS.angularVel, [gripper], ry.OT.sos, [1e1])


            M.solve()
            if not M.feasible:
                continue

            # Define the in between joint states when going for the grasp
            M5 = M.sub_motion(0)
            M5.no_collision([.3,.7], palm, box, margin=.05)
            M5.retract([.0, .2], gripper)
            M5.approach([.8, 1.], gripper)
            path1 = M5.solve()
            if not M5.feasible:
                continue

            # Define the in between joint states when placing the object
            M6 = M.sub_motion(1)
            M6.no_collision([], table, box)
            M6.no_collision([.2, .8], table, box, .12)
            path2 = M6.solve()
            if not M6.feasible:
                continue
            possible = True
            break
        if possible:
            break

    if not possible:
        return False
    
    bridge_paths.append(path1)
    bridge_paths.append(path2)
    
    C.setFrameState(frameStart)
    C.setJointState(qStart)


    C.view()
    robot = robex.Robot(C, on_real=False)
    robot.goHome(C)
    for i in range(3):
        box = f"box{i+1}"
        robot.execute_path_blocking(C, bridge_paths[i*2])
        robot.grasp(C)
        C.attach(gripper, box)
        
        robot.execute_path_blocking(C, bridge_paths[i*2+1])
        C.attach(table, box)
        robot.release(C)


    return bridge_paths


attempt_count = 100
for l in range(attempt_count):
    housePosition = [midpoint[0] + (np.random.random()*.2 -.1), midpoint[1] + (np.random.random()*.2 -.1)]
    if move_blocks(housePosition):
        continue
