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
ON_REAL = False

C.view()


def pivot(object_, info="pivot") -> bool:
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
    
    M1 = M.sub_motion(0, accumulated_collisions=False)
    M1.retractPush([.0, .15], gripper, .03)
    M1.approachPush([.85, 1.], gripper, .03)
    path1 = M1.solve()
    if not M1.feasible:
        return False
    
    # Set the predicted end frame state
    C.getFrame(object_).setQuaternion([np.pi*.5, np.pi*.5, 0, 0])

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
    
    C.getFrame(object_).setQuaternion([1, 0, 0, 0])

    # filler
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_point_to_point_motion(pose0, pose1)
    path = M.solve()
    if not M.feasible:
        return False
    
    M.play(C)
    C.getFrame(object_).setQuaternion([np.pi*.5, np.pi*.5, 0, 0])
    C.view(True)

    return True

pivot("box")
