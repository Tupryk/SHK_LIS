import numpy as np
import robotic as ry


def lookAtObj(objpos, bot, C):

    C.getFrame("predicted_obj").setPosition(objpos)
    q_now = C.getJointState()

    q_home = bot.get_qHome()

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1.], [.0, .0, .5])
    komo.addObjective([1.], ry.FS.position, ["l_gripper"], ry.OT.ineq, np.array([[.0, .0, -100.]]), [0, 0, 1])
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_home)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
        
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)


def getFilteredPointCloud(bot, ry_config, arena, z_cutoff=.68):
    bot.sync(ry_config, .0)
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
    
    cameraFrame = ry_config.getFrame("cameraWrist")

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


def getScannedObject(bot, ry_config, arena, visuals=True):

    points, _ = getFilteredPointCloud(bot, ry_config, arena)
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
        pclFrame = ry_config.getFrame("pcl")
        if not pclFrame:
            pclFrame = ry_config.addFrame('pcl')
            pclFrame.setPointCloud(np.array(points))
            pclFrame.setColor([0.,1.,0.]) #only to see it when overlaying with truth
            ry_config.view_recopyMeshes()

            ry_config.addFrame('mid_point') \
                .setPosition(midpoint) \
                .setShape(ry.ST.marker, size=[.2]) \
                .setColor([1, 1, 0])
        else:
            pclFrame.setPointCloud(np.array(points))
            ry_config.view_recopyMeshes()
            ry_config.getFrame('mid_point') \
                .setPosition(midpoint)
    
    return midpoint, points
