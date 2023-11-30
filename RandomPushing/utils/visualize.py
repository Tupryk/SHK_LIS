import numpy as np
import matplotlib.pyplot as plt

def viewPointCloud(points, colors=None):
    x = [v[0] for v in points]
    y = [v[1] for v in points]
    z = [v[2] for v in points]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if colors:
        for i, col in enumerate(colors): colors[i] = np.array(col)/255
        ax.scatter(x, y, z, c=colors, marker='o')
    else: ax.scatter(x, y, z, c='b', marker='o')
    
    plt.show()
