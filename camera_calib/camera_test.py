import robotic as ry
import matplotlib.pyplot as plt


C = ry.Config()
C.addFile(ry.raiPath('scenarios/justCam.g'))
bot = ry.BotOp(C, True)
bot.gripperClose(ry._left)
rgb, depth, points = bot.getImageDepthPcl('cameraFrame', False)

import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10,5))
axs = fig.subplots(1, 2)
axs[0].imshow(rgb)
axs[1].matshow(depth)
plt.show()
