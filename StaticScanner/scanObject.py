import robotic as ry
import numpy as np
import matplotlib.pyplot as plt
import cv2 


ON_REAL = False

def main():

    C = ry.Config()
    C.addFile(ry.raiPath('scenarios/pandaSingle_.g'))       # pandaSingle with added giraffe in my scenarios

    C.addFrame('predicted_obj') \
        .setShape(ry.ST.marker, size=[.1]) \
        .setPosition([0, .25, .7]) \
        .setColor([1, 0, 0])

    bot = ry.BotOp(C, ON_REAL)
    bot.home(C)

    bot.gripperMove(ry._left, .01, .4)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    
    C.view(True)



    q_now = C.getJointState()
    q_home = bot.get_qHome()

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 1., 0)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([1.], ry.FS.positionRel, ["predicted_obj", "cameraWrist"], ry.OT.eq, [1.], [.0, .0, .2])
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
        key=bot.sync(C, .1)
        if chr(key) == "q":
            print("Terminated (visual)")
            bot.home(C)
            del bot
            del C
            exit()


    rgb, depth, points = bot.getImageDepthPcl('cameraWrist', False)
    fig = plt.figure(figsize=(10,5))
    axs = fig.subplots(1, 2)
    axs[0].imshow(rgb)
    axs[1].matshow(depth)
    plt.show()

    # --- calculate color gradient
    # # Separate channels
    # red_channel = rgb[:,:,0]
    # green_channel = rgb[:,:,1]
    # blue_channel = rgb[:,:,2]

    # # Apply Sobel filter to each channel (adjust kernel size if needed)
    # sobel_red = cv2.Sobel(red_channel, cv2.CV_64F, 1, 0, ksize=3)  # Apply in x-direction
    # sobel_green = cv2.Sobel(green_channel, cv2.CV_64F, 1, 0, ksize=3)
    # sobel_blue = cv2.Sobel(blue_channel, cv2.CV_64F, 1, 0, ksize=3)

    # # Combine channels (optional, see alternative approach below)
    # magnitude_red, _ = cv2.cartToPolar(sobel_red, np.zeros_like(sobel_red))
    # magnitude_green, _ = cv2.cartToPolar(sobel_green, np.zeros_like(sobel_green))
    # magnitude_blue, _ = cv2.cartToPolar(sobel_blue, np.zeros_like(sobel_blue))
    # combined_gradient = (magnitude_red + magnitude_green + magnitude_blue) / 3  # Simple average

    # combined_gradient_norm = (combined_gradient - np.min(combined_gradient)) / (np.max(combined_gradient) - np.min(combined_gradient))
    # plt.imshow(combined_gradient_norm, cmap="gray")  # Use grayscale colorspace
    # plt.title("Combined Color Gradient")
    # plt.colorbar()  # Add a colorbar to visualize intensity levels
    # plt.show()

if __name__ == "__main__":
    main()