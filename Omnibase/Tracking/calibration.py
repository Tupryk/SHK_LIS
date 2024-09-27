import cv2
import numpy as np
import glob

# Termination criteria for corner sub-pix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on the chessboard pattern
chessboard_size = (9, 6)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0],
                       0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Load calibration images
images = glob.glob('markers/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)

        # Refine corner locations
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display corners
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Calibration', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the calibration results
np.savez('camera_calibration.npz',
         camera_matrix=camera_matrix,
         dist_coeffs=dist_coeffs)
