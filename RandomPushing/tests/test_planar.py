import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize
import functools

class PlanarRegressor:
    def __init__(self):
        self.stored_params = [0, 0, 0]

    def error(self, params, points):
        result = 0
        for (x,y,z) in points:
            plane_z = self.plane(x, y, params)
            diff = abs(plane_z - z)
            result += diff**2
        return result

    def fit(self, points):
        fun = functools.partial(self.error, points=points)
        res = scipy.optimize.minimize(fun, self.stored_params)

        self.stored_params = res.x
        return self

    def plane(self, x, y, params):
        a = params[0]
        b = params[1]
        c = params[2]
        return a*x + b*y + c
    
    def predict(self, xx, yy):
        a = self.stored_params[0]
        b = self.stored_params[1]
        c = self.stored_params[2]
        zz = []
        for i, _ in enumerate(xx):
            z = a*xx[i] + b*yy[i] + c
            zz.append(z)
        return np.array(zz)
    

grid_res = 16
xs, ys, zs = [], [], []
for i in range(grid_res):
    for j in range(grid_res):
        xs.append((1-i*2/grid_res))
        ys.append((1-j*2/grid_res))
        zs.append(xs[i]+ys[i])
        

xs = np.array(xs)
ys = np.array(ys)
zs = np.array(zs)
for i, _ in enumerate(zs):
    xs[i] += .1*(np.random.random()-0.5)
    ys[i] += .1*(np.random.random()-0.5)
    zs[i] += .1*(np.random.random()-0.5)

points = list(zip(xs, ys, zs))  # Convert zip iterator to a list

model = PlanarRegressor()
model.fit(points)

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(xs, ys, zs)

xx, yy = np.meshgrid(np.linspace(min(xs), max(xs), 10),
                     np.linspace(min(ys), max(ys), 10))
z = model.predict(xx.flatten(), yy.flatten())
z = z.reshape(xx.shape)
ax.plot_surface(xx, yy, z, alpha=0.2, color=[0, 1, 0])
plt.xlim(-1, 1)
plt.ylim(-1, 1)
plt.gca().set_aspect('equal')

plt.show()