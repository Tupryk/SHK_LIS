import openvr
import numpy as np
from time import *
import scipy.spatial.transform as st
# initialize OpenVR
openvr.init(openvr.VRApplication_Background)
 
# get the index of the controller
left_controller_index = openvr.k_unTrackedDeviceIndexInvalid
right_controller_index = openvr.k_unTrackedDeviceIndexInvalid
for i in range(openvr.k_unMaxTrackedDeviceCount):
    device_class = openvr.VRSystem().getTrackedDeviceClass(i)
    if device_class == openvr.TrackedDeviceClass_Controller and openvr.VRSystem().getControllerRoleForTrackedDeviceIndex(i) == openvr.TrackedControllerRole_LeftHand:
        left_controller_index = i
    if device_class == openvr.TrackedDeviceClass_Controller and openvr.VRSystem().getControllerRoleForTrackedDeviceIndex(i) == openvr.TrackedControllerRole_RightHand:
        right_controller_index = i
 
# get the controller's position and rotation

 
# extract the translation and rotation

 
# print the result
while True:
    pose = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
    left_pose = pose[left_controller_index].mDeviceToAbsoluteTracking
    right_pose = pose[right_controller_index].mDeviceToAbsoluteTracking
    
    #print("Left:", type(left_pose))
    #print(left_pose[:3,:3])


    rotation = np.array([
        [left_pose[0][0], left_pose[0][1], left_pose[0][2]],
        [left_pose[1][0], left_pose[1][1], left_pose[1][2]],
        [left_pose[2][0], left_pose[2][1], left_pose[2][2]],
    ])
    
    x_angle, y_angle, z_angle = st.Rotation.from_matrix(rotation).as_euler("xyz", degrees=True)

    #print(x_angle, y_angle, z_angle)
   
    print(left_pose[0][3], left_pose[1][3], left_pose[2][3])
   # print("Right:", right_pose)

    sleep(1)