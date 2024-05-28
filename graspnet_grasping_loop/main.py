import rowan
import numpy as np
import robotic as ry
from card_requester import request_card_processing


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle_tableCam.g'))
C.addFrame("obj0").setShape(ry.ST.cylinder, [.1, .04]).setPosition([0, .25, .7]).setContact(True).setMass(1.)

bot = ry.BotOp(C, False)
bot.home(C)

def catch_and_release(info: str="Catch and resease motion") -> bool:
    # Define the joint states at grasp and place
    C.addFrame("start_grasp_pose", "grasp_pose").setRelativePosition([.0, .0, .1])

    komo = ry.KOMO(C, 2., 32, 2, False)
    komo.addControlObjective([], order=0, scale=1e0)
    komo.addControlObjective([], order=1, scale=1e-1)
    komo.addControlObjective([], order=2, scale=1e-1)

    delta = C.getFrame("grasp_pose").getPosition() - \
            C.getFrame("start_grasp_pose").getPosition()
    
    delta /= np.linalg.norm(delta)
    mat = np.eye(3) - np.outer(delta, delta)
    komo.addObjective([1., 2.], ry.FS.positionDiff, ["l_gripper", "start_grasp_pose"], ry.OT.eq, mat)
    komo.addObjective([1.2, 2.], ry.FS.quaternionDiff, ["l_gripper", "grasp_pose"], ry.OT.eq, [1e1])

    komo.addObjective([1.], ry.FS.positionDiff, [gripper, "start_grasp_pose"], ry.OT.eq, [1e1])
    komo.addObjective([2.], ry.FS.positionDiff, [gripper, "grasp_pose"], ry.OT.eq, [1e1])

    sol = ry.NLP_Solver()
    sol.setProblem(komo.nlp())
    sol.setOptions(damping=1e-3, verbose=0, stopTolerance=1e-3, maxLambda=100., stopEvals=200)
    ret = sol.solve()
    komo.view(True)

    if not ret.feasible:
        print("Motion infeasible!!!")
        return False

    bot.move(komo.getPath(), [3.])
    while bot.getTimeToEnd() > .0:
        bot.sync(C)
    bot.gripperClose(ry._left)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    bot.home(C)
    bot.gripperMove(ry._left, .079)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)

    return True

for _ in range(3):
    rgb, depth, points = bot.getImageDepthPcl('cameraFrame', False)
    points = points.reshape(-1, 3)

    # grasp_pose = request_card_processing(points, "tcp://10.0.0.4:69420", verbose=1)
    grasp_pose = request_card_processing(points, "tcp://localhost:69420", verbose=1)

    rot_mat = grasp_pose[:3, :3]
    rot = rowan.from_matrix(rot_mat)
    rot = rowan.multiply(rot, np.array([.0, .0, 1., .0]))

    pos = grasp_pose[:, 3:][:3]
    pos = pos.T[0] + (rot_mat @ np.array([.0, .0, .015]))

    gpf = C.addFrame("grasp_pose", "cameraFrame").setShape(ry.ST.marker, [.2]).setRelativeQuaternion(rot).setRelativePosition(pos)
    gpf.setPosition(gpf.getPosition() - .1 * C.eval(ry.FS.vectorZ, ["grasp_pose"])[0])

    import manipulation as manip

    gripper = "l_gripper"
    table = "table"

    catch_and_release()
    C.view(True)
