import json
import numpy as np
import robotic as ry
from typing import Tuple, List
from RobotEnviroment.arenas import Arena


# TODO: Make code less repetitive and make the time for robot to move relative to the distance it has to travel.

def lookAtObjFromAngle(obj_pos: np.ndarray,
                  bot: ry.BotOp,
                  C: ry.Config,
                  angle: float,
                  radialDist: float=.2,
                  gripperHeight: float=.2,
                  speed: float=.2):

    C.getFrame("predicted_obj").setPosition(obj_pos) # This line should probaly not be in this function.
    q_now = C.getJointState()

    new_view = np.array([
        radialDist * np.sin(angle),
        radialDist * np.cos(angle),
        gripperHeight
    ])

    final_gripper_pos = new_view + obj_pos

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 3, 1., 0)

    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    komo.addObjective([1.], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], final_gripper_pos)
    komo.addObjective([1.], ry.FS.vectorZ, ["cameraWrist"], ry.OT.eq, [1.], -new_view/np.linalg.norm(new_view))
    komo.addObjective([1.], ry.FS.scalarProductYZ, ['l_gripper', 'table'], ry.OT.ineq, [-1e1], [0.])

    ret = ry.NLP_Solver().setProblem(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
    if not ret.feasible:
        print("Error while trying to look at object.")
        return
    
    dist_to_travel = np.linalg.norm(final_gripper_pos-C.getFrame("l_gripper").getPosition())

    bot.move(komo.getPath(), [dist_to_travel/speed])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)


def lookAtObjRandAngle(obj_pos: np.ndarray,
                  bot: ry.BotOp,
                  C: ry.Config,
                  radialDist: float=.2,
                  gripperHeight: float=.2):
    
    angle = np.random.random()*np.pi*2
    lookAtObjFromAngle(obj_pos, bot, C, angle, radialDist, gripperHeight)


def lookAtObj(obj_pos: np.ndarray,
              bot: ry.BotOp,
              C: ry.Config,
              dist2obj: float=.3):

    C.getFrame("predicted_obj").setPosition(obj_pos)
    q_now = C.getJointState()
    q_home = bot.get_qHome()

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1.], [.0, .0, dist2obj])
    komo.addObjective([1.], ry.FS.position, ["l_gripper"], ry.OT.ineq, np.array([[.0, .0, -100.]]), [0, 0, dist2obj*2.])
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_home)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
        
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)


def getFilteredPointCloud(bot: ry.BotOp,
                          C: ry.Config,
                          arena: Arena,
                          z_cutoff: float=.68) -> Tuple[List[float], List[float]]:
    bot.sync(C, .0)
    rgb, depth, points = bot.getImageDepthPcl('cameraWrist', False)

    new_rgb = []
    for lines in rgb:
        for c in lines: 
            new_rgb.append(c.tolist())
    rgb = new_rgb

    new_p = []
    for lines in points:
        for p in lines: 
            new_p.append(p.tolist())
    points = new_p

    new_rgb = []
    new_p = []
    for i, p in enumerate(points):
        if sum(p) != 0:
            new_rgb.append(rgb[i])
            new_p.append(p)
    points = np.array(new_p)
    
    cameraFrame = C.getFrame("cameraWrist")

    R, t = cameraFrame.getRotationMatrix(), cameraFrame.getPosition()

    points = points @ R.T
    points = points + np.tile(t.T, (points.shape[0], 1))

    objectpoints=[]
    colors = []
    for i, p in enumerate(points):
        if p[2] > (z_cutoff) and arena.point_in_arena(np.array(p)):
            objectpoints.append(p)
            colors.append(rgb[i])

    return objectpoints, colors


def getScannedObject(bot: ry.BotOp,
                     C: ry.Config,
                     arena: Arena,
                     visuals: bool=True) -> Tuple[np.ndarray, np.ndarray]:

    points, _ = getFilteredPointCloud(bot, C, arena)
    points = np.array(points)

    if not len(points):
        print("ERROR: Object not found!")
        return np.array([]), np.array([])

    min_coor = np.array([
        min([p[0] for p in points]),
        min([p[1] for p in points]),
        min([p[2] for p in points])
    ])

    max_coor = np.array([
        max([p[0] for p in points]),
        max([p[1] for p in points]),
        max([p[2] for p in points])
    ])

    midpoint = (max_coor+min_coor)/2
    
    if visuals:
        pclFrame = C.getFrame("pcl")
        if not pclFrame:
            pclFrame = C.addFrame('pcl')
            pclFrame.setPointCloud(np.array(points))
            pclFrame.setColor([0.,1.,0.]) #only to see it when overlaying with truth
            C.view_recopyMeshes()

            C.addFrame('mid_point') \
                .setPosition(midpoint) \
                .setShape(ry.ST.marker, size=[.2]) \
                .setColor([1, 1, 0])
        else:
            pclFrame.setPointCloud(np.array(points))
            C.view_recopyMeshes()
            C.getFrame('mid_point') \
                .setPosition(midpoint)
    
    return midpoint, points


def fullObjectScan(bot: ry.BotOp,
                   C: ry.Config,
                   obj_pos: np.ndarray,
                   arena: Arena,
                   gripper_height: float=.2,
                   gripper_distance: float=.2,
                   save_as: str="",
                   view_count: int=16) -> [[float]]:

    all_points = []
    angle_step = 2*np.pi/view_count
    for i in range(view_count):
        angle = angle_step * i
        new_view = np.array([
            gripper_distance * np.sin(angle),
            gripper_distance * np.cos(angle),
            gripper_height
        ])

        C.getFrame(f'view_point_{i}').setPosition(new_view+obj_pos)
        bot.sync(C)

        komo = ry.KOMO()
        komo.setConfig(C, True)
        komo.setTiming(1., 1, 1., 0)

        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

        komo.addObjective([1.], ry.FS.positionDiff, ['l_gripper', f'view_point_{i}'], ry.OT.eq, [1e1])
        komo.addObjective([1.], ry.FS.vectorZ, ["cameraWrist"], ry.OT.eq, [1.], -new_view/np.linalg.norm(new_view))

        ret = ry.NLP_Solver().setProblem(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        print(ret)
        bot.move(komo.getPath(), [3.])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        points, _ = getFilteredPointCloud(bot, C, arena)
        all_points.append([list(p) for p in points])
        getScannedObject(bot, C, arena)

    if len(save_as):
        json.dump(all_points, open(save_as, "w"))
    
    return all_points
