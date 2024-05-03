import numpy as np
import robotic as ry
from typing import List


def lookAtObj(bot: ry.BotOp, ry_config: ry.Config, objpos: np.ndarray):
    ry_config.getFrame("predicted_obj").setPosition(objpos)
    q_now = ry_config.getJointState()

    # bot.home(ry_config)
    q_home = bot.get_qHome()

    komo = ry.KOMO()
    komo.setConfig(ry_config, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    # Should be predicted obj instead of obj
    komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1.], [.0, .0, .5])
    komo.addObjective([1.], ry.FS.position, ["l_gripper"], ry.OT.ineq, np.array([[.0, .0, -100.]]), [0, 0, 1])
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_home)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)
        
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        key=bot.sync(ry_config, .1)
        if chr(key) == "q":
            print("Terminated (visual)")
            bot.home(ry_config)
            del bot
            del ry_config
            exit()


def getFilteredPointCloud(bot: ry.BotOp,
                          C: ry.Config,
                          origin: np.ndarray=np.array([-.5, 0, .69]),
                          z_cutoff: float=.68) -> List[np.ndarray]:
    
    rgb, depth, points = bot.getImageDepthPcl('cameraWrist', True)

    points = points.reshape(-1, 3)

    filtered_points = []
    for point in points:
        if point[2] > z_cutoff and np.linalg.norm(point[:2]-origin[:2]) <= .5:
            filtered_points.append(point)

    C.addFrame('pcl') \
        .setPointCloud(filtered_points) \
        .setColor([1.,0.,0.])
    
    return filtered_points
