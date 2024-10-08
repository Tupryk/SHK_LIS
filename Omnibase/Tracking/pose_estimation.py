import cv2
import numpy as np

# Load camera calibration parameters
camera_matrix = np.array([[615.1847534179688, 0, 320], [0, 615.1847534179688, 240], [0, 0, 1]])
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Choose the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_dict0 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
parameters =  cv2.aruco.DetectorParameters()

# Define the real-world marker size in meters
marker_length = 0.19

def list_cameras():
    available_cameras = []
    for i in range(30):  # Check the first 10 camera indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras

print("Available cameras:", list_cameras())


# Start video capture
cap = cv2.VideoCapture(10)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            # Draw axis for each marker
            cv2.drawFrameAxes(
                frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)

            # Print position and orientation
            print(f"Marker ID: {ids[i][0]}")
            print(f"Translation Vector (Position):\n{tvecs[i][0]}")
            print(f"Rotation Vector (Orientation):\n{rvecs[i][0]}\n")

    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray, aruco_dict0, parameters=parameters)

    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            # Draw axis for each marker
            cv2.drawFrameAxes(
                frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)

            # Print position and orientation
            print(f"Marker ID: {ids[i][0]}")
            print(f"Translation Vector (Position):\n{tvecs[i][0]}")
            print(f"Rotation Vector (Orientation):\n{rvecs[i][0]}\n")

    # Display the resulting frame
    cv2.imshow('ArUco Pose Estimation', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
