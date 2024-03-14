import robotic as ry
import manipulation as manip
import numpy as np
import robot_execution as robex

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.addFrame("box") \
    .setPosition([0, 0.4, 0.705]) \
    .setShape(ry.ST.ssBox, size=[0.04, 0.12, 0.04, 0.001]) \
    .setColor([1, 0, 0]) \
    .setContact(1) \
    .setMass(.1)

#C.delFrame("panda_collCameraWrist")

# for convenience, a few definitions:
gripper = "l_gripper"
palm = "l_palm"
table = "table"
ON_REAL = True


def pivot(object_, info="pivot") -> bool:
    qStart = C.getJointState()

    # pivot start
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_inverse_kinematics(accumulated_collisions=False)
    M.komo.addObjective([1.], ry.FS.positionRel, ["l_gripper", object_], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]), [0, 0, 0])
    M.komo.addObjective([1.], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
    M.komo.addObjective([1.], ry.FS.scalarProductXY, ["l_gripper", "table"], ry.OT.eq, [1e1], [0])
    M.komo.addObjective([1.], ry.FS.negDistance, ["l_gripper", object_], ry.OT.eq, [1e1], [-.01])
    M.no_collisions([1.], ["panda_collCameraWrist", "l_palm", object_], margin=.01)
    pose0 = M.solve()
    if not M.feasible:
        return False
    
    M1 = manip.ManipulationModelling(C, info, ['l_gripper'])
    M1.setup_point_to_point_motion(qStart, pose0[-1])
    path1 = M1.solve()
    if not M1.feasible:
        return False
    
    # Set the predicted end frame state
    C.setJointState(pose0[-1])
    C.getFrame(object_).setQuaternion([np.pi*.5, np.pi*.5, 0, 0])
    new_pos = C.getFrame(object_).getPosition() + np.array([0, .04, .04])
    C.getFrame(object_).setPosition(new_pos)

    # pivot end
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_inverse_kinematics()
    M.komo.addObjective([1.], ry.FS.positionRel, ["l_gripper", object_], ry.OT.eq, 1e1*np.array([[1,0,0],[0,0,1]]), [0, 0, 0])
    M.komo.addObjective([1.], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, 1])
    M.komo.addObjective([1.], ry.FS.scalarProductXY, ["l_gripper", "table"], ry.OT.eq, [1e1], [0])
    M.komo.addObjective([1.], ry.FS.negDistance, ["l_gripper", object_], ry.OT.eq, [1e1], [-.01])
    M.no_collisions([1.], ["panda_collCameraWrist", "l_palm", object_], margin=.01)
    pose1 = M.solve()
    if not M.feasible:
        return False
    
    C.addFrame("m") \
        .setPosition([0, 0.46, 0.705-.02]) \
        .setShape(ry.ST.marker, size=[0.04]) \
        .setColor([1, 0, 0])
    M2 = manip.ManipulationModelling(C, info, ['l_gripper'])
    M2.setup_point_to_point_motion(pose0[-1], pose1[-1])
    M2.komo.addObjective([], ry.FS.negDistance, ["l_gripper", "m"], ry.OT.eq, [-.1])
    path2 = M2.solve(verbose=0)
    if not M2.feasible:
        return False
    
    C.getFrame(object_).setQuaternion([1, 0, 0, 0])
    C.getFrame(object_).setPosition([0, 0.4, 0.705])
    C.setJointState(qStart)

    if ON_REAL:
        robot = robex.Robot(C, on_real=False)
        robot.grasp(C)
        robot.execute_path_blocking(C, path1)
        robot.execute_path_blocking(C, path2)
    else:
        M1.play(C, 4)
        M2.play(C, 4)

    C.getFrame(object_).setQuaternion([np.pi*.5, np.pi*.5, 0, 0])
    new_pos = C.getFrame(object_).getPosition() + np.array([0, .04, .04])
    C.getFrame(object_).setPosition(new_pos)

    return True

pivot("box")
