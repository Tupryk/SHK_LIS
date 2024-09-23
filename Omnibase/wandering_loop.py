import numpy as np
import robotic as ry

C = ry.Config()
ry.params_print()
C.addFile('./g_files/omnibase/omnibase.g')

C.addFrame("wall1") \
    .setShape(ry.ST.box, [5., .1, 1.]) \
    .setPosition([.0, 2.5, .5]) \
    .setColor([.5, .5, .7])

C.addFrame("wall2") \
    .setShape(ry.ST.box, [5., .1, 1.]) \
    .setPosition([.0, -2.5, .5]) \
    .setColor([.5, .5, .7])

C.addFrame("wall3") \
    .setShape(ry.ST.box, [.1, 5., 1.]) \
    .setPosition([2.5, .0, .5]) \
    .setColor([.5, .5, .7])

C.addFrame("wall4") \
    .setShape(ry.ST.box, [.1, 5., 1.]) \
    .setPosition([-2.5, .0, .5]) \
    .setColor([.5, .5, .7])

bot = ry.BotOp(C, False)
pclFrame = C.addFrame('pcl', 'cameraBase')

while True:
    omnibase_pos = C.getFrame("omnibase").getPosition()[:2]
    random_target = (np.random.random(size=2) - (np.ones((2,))*.5)) * 5. + omnibase_pos
    to_target_vec = random_target - omnibase_pos
    to_target_vec /= np.linalg.norm(to_target_vec)
    target_rot = np.arccos(np.clip(np.dot(to_target_vec, np.array([0, 1])), -1, 1))

    # Maybe do this without komo?
    komo = ry.KOMO(C, 1., 1, 0, False)
    komo.addObjective([1.], ry.FS.vectorY, ["omnibase"], ry.OT.eq, [1e1], [*(-to_target_vec), 0.])
    sol = ry.NLP_Solver()
    sol.setProblem(komo.nlp())
    sol.setOptions(damping=1e-3, verbose=0, stopTolerance=1e-3, maxLambda=100., stopEvals=200)
    ret = sol.solve()

    target_rot = komo.getPath()[0][2]
    q = [*omnibase_pos, target_rot]
    bot.moveTo(q)
    bot.wait(C)
    q = [*random_target, target_rot]

    bot.moveTo(q)
    while bot.getTimeToEnd() > 0:
        rgb, depth, points = bot.getImageDepthPcl("cameraBase")
        points = points.reshape(-1, 3)
        points = points[points[:, 1] < 0.2]
        points = points[points[:, 2] != 0.]
        pclFrame.setPointCloud(points)
        pclFrame.setColor([1., 0., 0.])
        C.view()
        bot.sync(C)

        if points[:, 2].min() < .7:
            print("OH FUCK, OH GOD, I'M GOING TO CRASH, AAAAAAAAAAAAAAAHHHHHHH!!!!!!")
            bot.stop(C)
