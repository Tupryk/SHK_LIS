import robotic as ry
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from helperFunctions import filter_points_outside_cuboid, generate_path_on_partial_spherical_surface


ON_REAL = False


def main():

    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle_.g')) # pandaSingle with added giraffe in my scenarios

    C.addFrame('predicted_obj') \
        .setShape(ry.ST.marker, size=[.1]) \
        .setPosition([0, .25, .7]) \
        .setColor([1, 0, 0])

    bot = ry.BotOp(C, ON_REAL)
    bot.home(C)

    bot.gripperMove(ry._left, .01, .4)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    


    # -- TODO clean KOMO --

    sphere_path = generate_path_on_partial_spherical_surface(6, (np.radians(20), np.radians(60)), .2, (0, .25, .7))
    C.view(True)

    q_now = C.getJointState()
    q_home = bot.get_qHome()

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1.], [.0, .0, .2])
    komo.addObjective([1.], ry.FS.position, ["cameraWrist"], ry.OT.eq, [1.], sphere_path[0])

    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_home)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)
        
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        key=bot.sync(C, .1)
        if chr(key) == "q":
            print("Terminated (visual)")
            bot.home(C)
            del bot
            del C
            exit()

    _, __, points = bot.getImageDepthPcl('cameraWrist', False)
    cameraFrame = C.getFrame("cameraWrist")
    R, t = cameraFrame.getRotationMatrix(), cameraFrame.getPosition()

    # points to world coords and to N x 3
    points = points.reshape(points.shape[0]*points.shape[1],points.shape[2])
    points = points @ R.T
    points = points + np.tile(t.T, (points.shape[0], 1))

    # filter points on the table and from robot base  
    points = filter_points_outside_cuboid(points, (0, -.225, .75), (.275, .275, .4))  
    filtered_points = points[points[:, 2] >= .659]+ np.array([.3, 0,0])
    
    pclFrame = C.addFrame("pcl")

    pclFrame.setPointCloud(filtered_points)
    C.view_recopyMeshes()
    C.view(True)

if __name__ == "__main__":
    main()