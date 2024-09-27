import cv2
import numpy as np

# Load camera calibration parameters
camera_matrix = np.array([[615.1847534179688, 0, 320.5], [0, 615.1847534179688, 240.5], [0, 0, 1]])

dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters =  cv2.aruco.DetectorParameters()

# Start video capture
cap = cv2.VideoCapture(0)  # Use the appropriate camera index

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

    # Display the resulting frame
    cv2.imshow('ArUco Detection', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
