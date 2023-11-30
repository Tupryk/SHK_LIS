from robotic import ry
import numpy as np
import matplotlib.pyplot as plt
from config import setup_config, startup_robot
from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter

WAYPOINTS = 6
INITIAL_OBJ_POS = [.0, .05, .69]
OBJ_HEIGHT = .08

if __name__ == "__main__":

    #-- load parameters, typically automatically from 'rai.cfg'
    ry.params_print()
    verbose = 1

    #-- define a configuration
    C = setup_config(WAYPOINTS, INITIAL_OBJ_POS)
    bot = startup_robot(C, False)

    bot.home(C)
    # Robot gripper has to be looking down
    opos = C.getFrame("obj").getPosition()
    gpos = C.getFrame("l_gripper").getPosition()

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([1.], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e4], gpos-opos)
    komo.addObjective([1.], ry.FS.positionDiff, ["l_gripper", "obj"], ry.OT.eq, [1e4], [0.5*(gpos-opos)])
    #komo.addObjective([2.], ry.FS.distance, ["l_gripper", "obj"], ry.OT.eq, [1e4], [.4])


    #komo.addObjective([1.], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], (opos-gpos)*-1)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)
        
    bot.moveTo(komo.getPath()[0], 1., False)
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)

    image_rgb, depth = bot.getImageAndDepth('camera')
    plt.imshow(image_rgb)
    plt.show()
    plt.imshow(depth)
    plt.show()

    image_gray = color.gray2rgb(depth)
    plt.imshow(image_gray)
    plt.show()

    #image_gray = color.rgb2gray(image_rgb)
    edges = canny(image_gray, sigma=1.0,
                low_threshold=0.1, high_threshold=0.6)

    plt.imshow(edges)
    plt.show()

    # Perform a Hough Transform
    # The accuracy corresponds to the bin size of a major axis.
    # The value is chosen in order to get a single high accumulator.
    # The threshold eliminates low accumulators
    result = hough_ellipse(edges, accuracy=20, threshold=250,
                        min_size=100, max_size=120)
    result.sort(order='accumulator')

    # Estimated parameters for the ellipse
    best = list(result[-1])
    yc, xc, a, b = (int(round(x)) for x in best[1:5])
    orientation = best[5]

    # Draw the ellipse on the original image
    cy, cx = ellipse_perimeter(yc, xc, a, b, orientation)
    image_gray[cy, cx] = (0, 0, 255)
    # Draw the edge (white) and the resulting ellipse (red)
    edges = color.gray2rgb(img_as_ubyte(edges))
    edges[cy, cx] = (250, 0, 0)

    fig2, (ax1, ax2) = plt.subplots(ncols=2, nrows=1, figsize=(8, 4),
                                    sharex=True, sharey=True)

    ax1.set_title('Original picture')
    ax1.imshow(image_gray)

    ax2.set_title('Edge (white) and result (red)')
    ax2.imshow(edges)

    plt.show()

