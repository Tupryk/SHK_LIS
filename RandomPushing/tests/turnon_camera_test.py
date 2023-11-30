from robotic import ry
import numpy as np
import time
import matplotlib.pyplot as plt
import cv2

ON_TRUE = False

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

obj = C.addFrame('obj')
obj.setPose('t(0. 0.1 0.8)')
obj.setShape(ry.ST.ssBox, size=[.05,.05,.05,.005])
obj.setColor([1,.0,0])
obj.setMass(.1)
obj.setContact(True)
C.view()

bot = ry.BotOp(C, ON_TRUE)
bot.home(C)
c = 0 
while True:
    c += 1
    bot.sync(C, .1)
    if bot.getKeyPressed()==ord('q'):
       break
    rgb, depth, points = bot.getImageDepthPcl('cameraWrist', False)
    
    if c % 35 == 0:
        fig = plt.figure(figsize=(10,5))
        axs = fig.subplots(1, 2)
        axs[0].imshow(rgb)
        axs[1].matshow(depth)
        plt.show()

    # If camera is not attached, we should get black pixels 
    total_pixel_sum = np.sum(rgb)

    # Check if the total sum is zero (indicating a black image)
    if total_pixel_sum == 0:
        print("Camera records a black image. Check light and camera sensor. ")
    elif total_pixel_sum == (255 * rgb.shape[2]) * rgb.shape[0] * rgb.shape[1]:
        print("Camera records a white image. Check light and camera sensor. ")
    else:
        print("Camera working.")



