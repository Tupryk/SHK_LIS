import cv2
import rowan
import numpy as np
import pandas as pd
import robotic as ry
from scipy.spatial.transform import Rotation


object_points = np.array([
    [0.0, 0.1, 0.89],
    [0.15, 0.4, 0.9],
    [-0.1, 0.25, 0.75],
    [0.1, 0.4, 0.8],
    [-0.1, 0.0, 0.86],
    [0.0, 0.3, 0.77],
    [0.12, 0.37, 0.71],
    [-0.1, 0.33, 0.72],
    [-0.1, 0.2, 0.76]
], dtype=np.float32)

df = pd.read_csv('red_dot_coordinates.csv')

image_points = df[['X', 'Y']].values.astype(np.float32)

camera_matrix = np.array([
    [322.2, 0, 320],
    [0, 322.2, 180],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((4, 1), dtype=np.float32)

success, rvecs, tvecs = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

np_rodrigues = np.asarray(rvecs[:,:],np.float64)
rmat = cv2.Rodrigues(np_rodrigues)[0]
camera_position = -np.matrix(rmat).T @ np.matrix(tvecs)

r = Rotation.from_rotvec([rvecs[0][0],rvecs[1][0],rvecs[2][0]])
rot = r.as_euler('xyz', degrees=True)

euler_angles_deg = np.array([180-rot[0], rot[1], rot[2]])
euler_angles_rad = np.radians(euler_angles_deg)
rot = rowan.from_euler(*euler_angles_rad, 'xyz')
rotation_quaternion = np.array([0, 0, 1, 0])
rot = rowan.multiply(rotation_quaternion, rot)
print(rot)

C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle_tableCam.g'))
C.addFrame("camera_prediction").setShape(ry.ST.marker, [.03]).setPosition(camera_position).setQuaternion(rot)
C.view(True)
