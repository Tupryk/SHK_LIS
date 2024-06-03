import cv2
import rowan
import numpy as np
import pandas as pd
import robotic as ry
from scipy.spatial.transform import Rotation


df = pd.read_csv('3dpoints.csv')
object_points = df[['X', 'Y', 'Z']].to_numpy(dtype=np.float32)

df = pd.read_csv('red_dot_coordinates.csv')

image_points = df[['X', 'Y']].values.astype(np.float32)
new_object_points = []
tmp = 0
for i, image in enumerate(df["Image"]):
    if image == f"image{i+tmp}.png":
        new_object_points.append(object_points[i+tmp])
    else:
        tmp += 1
        new_object_points.append(object_points[i+tmp])
object_points = np.array(new_object_points)

print(len(image_points))
print(len(object_points))

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
C.addFrame("camera_prediction").setShape(ry.ST.marker, [.09]).setPosition(camera_position).setQuaternion(rot)
C.view(True)

bot = ry.BotOp(C, True)
rgb, depth, points = bot.getImageDepthPcl('cameraFrame', True)
R, t = rowan.to_matrix(C.getFrame("camera_prediction").getQuaternion()), C.getFrame("camera_prediction").getPosition()

points = points.reshape(-1, 3)
points = points @ R.T
points = points + np.tile(t.T, (points.shape[0], 1))

pclFrame = C.addFrame('pcl')
pclFrame.setPointCloud(np.array(points))
pclFrame.setColor([0.,1.,0.]) #only to see it when overlaying with truth
C.view_recopyMeshes()
C.view(True)