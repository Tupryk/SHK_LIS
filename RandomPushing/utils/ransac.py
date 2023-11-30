from copy import copy
import numpy as np
from numpy.random import default_rng
import scipy.optimize
import functools
import robotic as ry
import random

rng = default_rng()


#RANSAC implementationa
class RANSAC:
    def __init__(self, n=8, k=100, t=0.03, d=10, model=None, loss=None, metric=None):
        self.n = n              # `n`: Minimum number of data points to estimate parameters
        self.k = k              # `k`: Maximum iterations allowed
        self.t = t              # `t`: Threshold value to determine if points are fit well
        self.d = d              # `d`: Number of close data points required to assert model fits well
        self.model = model      # `model`: class implementing `fit` and `predict`
        self.loss = loss        # `loss`: function of `y_true` and `y_pred` that returns a vector
        self.metric = metric    # `metric`: function of `y_true` and `y_pred` and returns a float
        self.best_fit = None
        self.best_error = np.inf

    def fit(self, x, y, z):
        for _ in range(self.k):
            ids = rng.permutation(x.shape[0])

            maybe_inliers = ids[: self.n]
            maybe_model = copy(self.model).fit(list(zip(x[maybe_inliers], y[maybe_inliers], z[maybe_inliers])))

            thresholded = (
                self.loss(z[ids][self.n :], maybe_model.predict(x[ids][self.n :], y[ids][self.n :]))
                < self.t
            )

            inlier_ids = ids[self.n :][np.flatnonzero(thresholded).flatten()]

            if inlier_ids.size > self.d:
                inlier_points = np.hstack([maybe_inliers, inlier_ids])
                better_model = copy(self.model).fit(list(zip(x[inlier_points], y[inlier_points], z[inlier_points])))

                this_error = self.metric(
                    z[inlier_points], better_model.predict(x[inlier_points], y[inlier_points])
                )

                if this_error < self.best_error:
                    self.best_error = this_error
                    self.best_fit = maybe_model

        return self

    def predict(self, x, y):
        return self.best_fit.predict(x,y)

def square_error_loss(z_true, z_pred):
    return (z_true - z_pred) ** 2


def mean_square_error(z_true, z_pred):
    return np.sum(square_error_loss(z_true, z_pred)) / z_true.shape[0]


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
    
def point_above_plane(point, normal, plane_point, offset=.01):
    vector_to_point = (np.array(point)-np.array([.0, .0, offset])) - np.array(plane_point)
    dot_product = np.dot(vector_to_point, normal)
    return dot_product < 0

def get_plane_from_points(points, ry_config):
    """
    Should be called at program start with no object on table
    """
    points = random.sample(points, 256)

    regressor = RANSAC(model=PlanarRegressor(), loss=square_error_loss, metric=mean_square_error)

    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])
    z = np.array([p[2] for p in points])

    regressor.fit(x, y, z)

    x = np.array([-.5, .5, .0])
    y = np.array([.5, .5, -.5])
    z = regressor.predict(x, y)

    points = [0, 0, 0]
    for i in range(3):
        points[i] = np.array([x[i], y[i], z[i]])

    mid_point = np.array([.0, .0, .0])
    for i in range(3):
        mid_point += points[i]
    mid_point /= 3

    normal = np.cross(points[1] - points[0], points[2] - points[0])

    table_ball = ry_config.getFrame("predicted_table_point0")
    if not table_ball:
        for i in range(3):
            ry_config.addFrame(f"predicted_table_point{i}") \
                .setShape(ry.ST.sphere, size=[.05]) \
                .setPosition(points[i]) \
                .setColor([0, 0, 1])
    else:
        for i in range(3):
            ry_config.getFrame(f"predicted_table_point{i}").setPosition(points[i])

    return normal, mid_point
