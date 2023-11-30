import numpy as np
import matplotlib.pyplot as plt

from utils import RANSAC, PlanarRegressor, square_error_loss, mean_square_error

if __name__ == "__main__":
    regressor = RANSAC(model=PlanarRegressor(), loss=square_error_loss, metric=mean_square_error)

    grid_res = 16
    x, y, z = [], [], []
    for i in range(grid_res):
        for j in range(grid_res):
            x.append(1-i*2/grid_res)
            y.append(1-j*2/grid_res)
            z.append(x[i]+y[i])

    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    for i, _ in enumerate(z):
        x[i] += .08*(np.random.random()-0.5)
        y[i] += .08*(np.random.random()-0.5)
        if(np.random.random()<.25):
            z[i] += (np.random.random()-0.5)

        z[i] += .02*(np.random.random()-0.5)

    regressor.fit(x, y, z)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x, y, z)


    xx, yy = np.meshgrid([-1,1], [-1,1])
    z = regressor.predict(xx, yy)
    print(xx)
    print(yy)
    print(z)
    ax.plot_surface(xx, yy, z, alpha=0.2, color=[0,1,1])
    plt.xlim(-1, 1)
    plt.ylim(-1, 1)
    plt.gca().set_aspect('equal')
    plt.show()