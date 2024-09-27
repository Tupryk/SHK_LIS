import cv2
import rowan
import numpy as np
import robotic as ry


# Real world values
marker_length = 0.19  # m
box_dims = 0.21  # m
camera_pos = np.array([0., 0., 0.])
box_offset = np.array([0., .2, .5])
front_id = 42
back_id = 23
left_id = 1
right_id = 24

# Camera
camera_matrix = np.array([[615.1847534179688, 0, 320], [0, 615.1847534179688, 240], [0, 0, 1]])
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
camera_idx = 10

def list_cameras():
    available_cameras = []
    for i in range(30):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras

print("Available cameras:", list_cameras())

cap = cv2.VideoCapture(camera_idx)

# Aruco dictionaries
aruco_back_left = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_front_right = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
parameters =  cv2.aruco.DetectorParameters()

# Estimated configuration
C = ry.Config()

real_box_pos = np.array([0., 0., 1.])
base_pos = np.array([0., 0., 1.])
box_frame = C.addFrame("tracker_box") \
    .setShape(ry.ST.ssBox, [box_dims, box_dims, box_dims, .02]) \
    .setColor([1., .5, 0.]) \
    .setPosition([0., 0., .6])
original_z = box_frame.getPosition()[2]

base_frame = C.addFrame("omnibase", "tracker_box") \
    .setShape(ry.ST.cylinder, [.04, .3]) \
    .setColor([.8, .8, .8]) \
    .setRelativePosition(-box_offset)
C.addFrame("direction", "omnibase") \
    .setShape(ry.ST.box, [.04, .04, .04]) \
    .setColor([.8, .0, .0]) \
    .setRelativePosition([0., -.3, .02])

C.addFrame("stick", "omnibase") \
    .setShape(ry.ST.box, [.04, .04, .5]) \
    .setColor([.0, .0, 1.]) \
    .setRelativePosition([0., .2, .25])


aruco_frames = []
for i in range(4):
    aruco_frame = C.addFrame(f"aruco_frame_{i}") \
        .setShape(ry.ST.marker, [.1])
    aruco_frames.append(aruco_frame)

C.view()

# Main loop
while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Get aruco frames
    corners_bl, ids_bl, rejected_bl = cv2.aruco.detectMarkers(
        gray, aruco_back_left, parameters=parameters)
    corners_fr, ids_fr, rejected_fr = cv2.aruco.detectMarkers(
        gray, aruco_front_right, parameters=parameters)
    
    ids = []
    corners = []
    if ids_bl is not None:
        ids.extend(ids_bl)
        corners.extend(corners_bl)
    if ids_fr is not None:
        ids.extend(ids_fr)
        corners.extend(corners_fr)
    ids = np.array(ids)
    corners = np.array(corners)

    if len(ids):
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs)

        pos_avg = np.zeros((3,))
        rot_avg = 0
        for i, id in enumerate(ids):
            cv2.drawFrameAxes(
                frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)

            pos = np.array([tvecs[i][0][0], tvecs[i][0][2], tvecs[i][0][1]])
            rot = rvecs[i][0]
            aruco_frames[i].setPosition(pos) \
                .setQuaternion(rowan.from_euler(*rot))
            
            box_pos = pos
            if id == front_id:
                box_pos[1] -= box_dims*.5
                box_rot = -(rot[1] + np.pi)
            elif id == back_id:
                box_pos[1] += box_dims*.5
                box_rot = rot[2] - np.pi
            elif id == left_id:
                box_pos[0] += box_dims*.5
                box_rot = rot[2] - np.pi*.5
            elif id == right_id:
                box_pos[0] -= box_dims*.5
                box_rot = rot[2] + np.pi*.5

            pos_avg += box_pos
            rot_avg += box_rot
        pos_avg /= len(ids)
        rot_avg /= len(ids)
        pos_avg[2] = original_z
        rot_avg = np.array([rot_avg, 0, 0])
        # rot_avg[0] = 0.
        # rot_avg[1] = 0.
        box_frame.setPosition(pos_avg) \
            .setQuaternion(rowan.from_euler(*rot_avg))
    
    # Hide unnecessary markers
    for i in range(len(ids), 4):
        aruco_frames[i].setPosition([0, 0, 0]) \
            .setQuaternion(rowan.from_euler(0, 0, 0))

    # Display stuff
    C.view()
    cv2.imshow('ArUco Pose Estimation', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

del C
cap.release()
cv2.destroyAllWindows()
