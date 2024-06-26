import cv2
import rowan
import numpy as np
import pandas as pd
import robotic as ry
import matplotlib.pyplot as plt




df = pd.read_csv('3dpoints.csv')
object_points = df[['X', 'Y', 'Z']].to_numpy(dtype=np.float32)
# Corresponding 2D points in the image plane
df = pd.read_csv('red_dot_coordinates.csv')

# Extract the X and Y columns
try: 
    image_points = df[['X', 'Y']].values.astype(np.float32)
except ValueError:
    #TODO delete corresponding line in csv then
    pass

new_object_points = []
tmp = 0
for i, image in enumerate(df["Image"]):
    if image == f"image{i+tmp}.png":
        new_object_points.append(object_points[i+tmp])
    else:
        tmp += 1
        new_object_points.append(object_points[i+tmp])
object_points = np.array(new_object_points)


# Camera intrinsic parameters
camera_matrix = np.array([
    [322.2, 0, 320],
    [0, 322.2, 180],
    [0, 0, 1]
], dtype=np.float32)

# Assuming no lens distortion for simplicity; otherwise, provide distortion coefficients
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

# Solve for the rotation and translation vectors
success, rotation_vector, translation_vector = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

if success:
    print("Rotation Vector:\n", rotation_vector)
    print("Translation Vector:\n", translation_vector)

    # Convert rotation vector to rotation matrix (optional)
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    print("Rotation Matrix:\n", rotation_matrix)

    # Project the 3D points back onto the image plane
    projected_points, _ = cv2.projectPoints(object_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

    # Visualize the original and projected points
    image = np.zeros((360, 640, 3), dtype=np.uint8)  # Placeholder for an actual image
    for i, (orig, proj) in enumerate(zip(image_points, projected_points)):
        orig = tuple(map(int, orig))
        proj = tuple(map(int, proj.ravel()))
        cv2.circle(image, orig, 5, (0, 255, 0), -1)  # Original points in green
        cv2.circle(image, proj, 5, (0, 0, 255), -1)  # Projected points in red
        cv2.line(image, orig, proj, (255, 0, 0), 1)  # Line between them in blue

    # Show the image with points
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title('Original (Green) vs Projected (Red) Points')
    plt.show()

else:
    print("SolvePnP failed to find a solution.")


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle_tableCam.g'))
C.addFrame("camera_prediction").setShape(ry.ST.marker, [.03]).setPosition(translation_vector).setQuaternion(rowan.from_matrix(rotation_matrix))
print(C.getFrame("cameraFrame").getPosition())
C.view(True)
