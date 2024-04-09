import robotic as ry
import matplotlib.pyplot as plt
import numpy as np
from helperFunctions import filter_points_outside_cuboid, generate_path_on_partial_spherical_surface
import open3d as o3d 

ON_REAL = False


def main():

    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable_.g'))

    C.addFrame('predicted_obj') \
        .setShape(ry.ST.marker, size=[.1]) \
        .setPosition([.1, .14, .59]) \
        .setColor([1, 0, 0])

    bot = ry.BotOp(C, ON_REAL)
    
    sphere_path = generate_path_on_partial_spherical_surface(3, (np.radians(40), np.radians(60)), (3*np.pi/2,2*np.pi), .4, (.1, .2, .67), visualize=True, C=C)

    
    C.view(True)

    

    #experimental 
    q_now = C.getJointState()
    q_home = bot.get_qHome()

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1, 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([1], ry.FS.position, ["l_gripper"], ry.OT.eq, [1.], C.getFrame("predicted_obj").getPosition())
    komo.addObjective([1], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1.], [0,0,1])
    komo.addObjective([1], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1.], [1/np.sqrt(2), -1/np.sqrt(2),0])

    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_home)
    komo.addObjective([], ry.FS.qItself, [], ry.OT.sos, [.1], q_now)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        key=bot.sync(C, .1)
        if chr(key) == "q":
            print("Terminated (visual)")
            bot.home(C)
            del bot
            del C
            exit()

    C.view(True)
    #C.attach("obj_mesh", "l_gripper")
    #C.attach("obj", "l_gripper")
    bot.gripperMove(ry._left, .0)


    while not bot.gripperDone(ry._left):
        bot.sync(C)
    print( bot.getGripperPos(ry._left))

    bot.home(C)


if __name__ == "__main__":
    main()