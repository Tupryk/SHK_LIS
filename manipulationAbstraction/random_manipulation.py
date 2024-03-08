import robotic as ry
import manipulation as manip
import numpy as np
import random
import robot_execution as robex

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.addFrame("box", "table") \
    .setJoint(ry.JT.rigid) \
    .setShape(ry.ST.ssBox, [.15,.06,.06,.005]) \
    .setRelativePosition([-.0,.3-.055,.095]) \
    .setContact(1) \
    .setMass(.1)

# C.addFrame("obstacle", "table") \
#     .setShape(ry.ST.ssBox, [.06,.15,.06,.005]) \
#     .setColor([.1]) \
#     .setRelativePosition([-.15,.3-.055,.095]) \
#     .setContact(1)

C.delFrame("panda_collCameraWrist")

# for convenience, a few definitions:
qHome = C.getJointState()
gripper = "l_gripper"
palm = "l_palm"
box = "box"
table = "table"
boxSize = C.frame(box).getSize()

C.view()

def pick_place(graspDirection, placeDirection, placePosition, info) -> bool:
    # Define the joint states at grasp and place
    M = manip.ManipulationModelling(C, info, helpers=[gripper])
    M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1)
    M.grasp_box(1., gripper, box, palm, graspDirection)
    M.place_box(2., box, table, palm, placeDirection)
    M.target_relative_xy_position(2., box, table, placePosition)
    M.solve()
    if not M.feasible:
        return False

    # Define the in between joint states when going for the grasp
    M1 = M.sub_motion(0)
    M1.no_collision([.3,.7], palm, box, margin=.05)
    M1.retract([.0, .2], gripper)
    M1.approach([.8, 1.], gripper)
    path1 = M1.solve()
    if not M1.feasible:
        return False

    # Define the in between joint states when placing the object
    M2 = M.sub_motion(1)
    M2.no_collision([], table, box)
    path2 = M2.solve()
    if not M2.feasible:
        return False

    M1.play(C)
    C.attach(gripper, box)
    M2.play(C)
    C.attach(table, box)

    # robot = robex.Robot(C)
    # robot.execute_path_blocking(C, path1)
    # robot.grasp(C)
    # robot.execute_path_blocking(C, path2)

    return True


def push(pushPosition, info) -> bool:
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_pick_and_place_waypoints(gripper, box, 1e-1)
    M.straight_push([1.,2.], box, gripper, table)
    M.komo.addObjective([2.], ry.FS.position, [box], ry.OT.eq, 1e1*np.array([[1,0,0],[0,1,0]]), pushPosition)
    M.solve()
    if not M.feasible:
        return False

    M1 = M.sub_motion(0)
    M1.retractPush([.0, .15], gripper, .03)
    M1.approachPush([.85, 1.], gripper, .03)
    M1.no_collision([.15,.85], box, "l_finger1", .02)
    M1.no_collision([.15,.85], box, "l_finger2", .02)
    M1.no_collision([.15,.85], box, 'l_palm', .02)
    M1.no_collision([], table, "l_finger1", .0)
    M1.no_collision([], table, "l_finger2", .0)
    path1 = M1.solve()
    if not M1.feasible:
        return False

    M2 = M.sub_motion(1)
    M2.komo.addObjective([], ry.FS.positionRel, [gripper, '_push_start'], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]))
    path2 = M2.solve()
    if not M2.feasible:
         return False

    M1.play(C, 1.)
    C.attach(gripper, box)
    M2.play(C, 1.)
    C.attach(table, box)

    return True


attempt_count = 20
data = []
for l in range(attempt_count):

    # action = np.random.choice(["grasp", "push"])
    action = np.random.choice(["push"])
    if action == "grasp":
        graspDirection = random.choice(['y', 'z']) #'x' not possible: box too large
        placeDirection = random.choice(['x', 'y', 'z', 'xNeg', 'yNeg', 'zNeg'])
        placePosition = [.2 + np.random.random()*.2 -.1, .3 + np.random.random()*.2 -.1]
        info = f'placement {l}: grasp {graspDirection} place {placeDirection}'
        success = pick_place(graspDirection, placeDirection, placePosition, info)

    elif action == "push":
        pushPosition = .4*np.random.rand(3) - .2+np.array([.0,.3,.0])
        info = f'push {l}'
        success = push(pushPosition, info)

    else:
        raise Exception(f'Action "{action}" is not defined!')
    
    data.append({"action": action, "success": success})
print(data)
