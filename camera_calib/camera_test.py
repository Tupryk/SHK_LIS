import robotic as ry
import matplotlib.pyplot as plt

# Translation Vector:
#  [[-0.40824952]
#  [ 0.70033434]
#  [ 0.68738615]]
# Rotation Matrix:
#  [[ 0.18391336  0.98066547  0.06686638]
#  [ 0.2340221   0.02238537 -0.97197353]
#  [-0.95467771  0.19440713 -0.22538043]]


C = ry.Config()
C.addFile(ry.raiPath('scenarios/justCam.g'))
bot = ry.BotOp(C, False)

C.addFrame(f'way'). setShape(ry.ST.marker, [5]) .setPosition([-.40824952, .7, .687])

rgb, depth, points = bot.getImageDepthPcl('cameraFrame', False)

import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10,5))
axs = fig.subplots(1, 2)
axs[0].imshow(rgb)
axs[1].matshow(depth)
plt.show()
