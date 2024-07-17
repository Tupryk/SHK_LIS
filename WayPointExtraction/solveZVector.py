import numpy as np
from scipy.interpolate import BSpline
from scipy.linalg import lstsq
import robotic as ry 
import time

def compute_base_spline_matrix(num_path_points, num_basis_functions, degree):
    t_path = np.linspace(0, 1, num_path_points)
    knot_vector = np.concatenate(([0] * degree, np.linspace(0, 1, num_basis_functions - degree + 1), [1] * degree))

    basis_matrix = np.zeros((num_path_points, num_basis_functions))

    for i in range(num_basis_functions):
        coeff = np.zeros(num_basis_functions)
        coeff[i] = 1
        spline = BSpline(knot_vector, coeff, degree)
        basis_matrix[:, i] = spline(t_path)
    
    return basis_matrix

def solve_least_squares(B, q):
    z, _, _, _ = lstsq(B, q)
    return z


def solveZinQSpace(q, num_cPoints, degree=2, verbose=2):
    num_path_points = len(q)  
    num_cPoints = num_cPoints
    degree = 2

    B = compute_base_spline_matrix(num_path_points, num_cPoints, degree)

    z = solve_least_squares(B, q)

    if verbose & 1:
        print("Control Points (z):")
        print(z)
    if verbose & 2:
        for i, q in enumerate(z):
            C.setJointState(q)
            C.addFrame(f'way{i}'). setShape(ry.ST.marker, [.1]) .setPosition(C.getFrame("l_gripper").getPosition())
            C.view(False)
            time.sleep(.5)
        C.view(True)


    return z

def solveZinTaskSpace(C, q, num_cPoints, degree=2, verbose=2):
    s = []      # end effector states
    for i in q:
        C.setJointState(i)
        s.append(C.getFrame("l_gripper").getPosition())
    
    num_path_points = len(s) 
    num_cPoints = num_cPoints
    degree = 2

    B = compute_base_spline_matrix(num_path_points, num_cPoints, degree)

    z = solve_least_squares(B, s)

    if verbose & 1:
        print("Control Points (z):")
        print(z)
    if verbose & 2:
        for i, q in enumerate(z):
            C.addFrame(f'task_way{i}'). setShape(ry.ST.marker, [.1]) .setPosition(q)
            C.view(False)
            time.sleep(.5)
        C.view(True)

    return z

q = np.load("logs/only_qReverse.npy")

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

z = solveZinQSpace(q, 10)
#z = solveZinTaskSpace(C, q, 10)

