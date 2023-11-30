import openvr
import numpy as np
from robotic import ry
from robot_functions import update_robot
import scipy.spatial.transform as st
from scipy.spatial.transform import Rotation as R
from time import sleep

ry.params_add({"physx/mototKp": 10_000., "physx/motorKd": 1_000.})
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.addFrame("marker") \
    .setPosition([0, 0, 0]) \
    .setShape(ry.ST.marker, size=[0.2]) \
    .setColor([1., .0, .0, 0.3])

C.addFrame("pen") \
    .setPosition([-0.25, .1, .675]) \
    .setShape(ry.ST.ssBox, size=[.02, .15, .02, .005]) \
    .setColor([1., .5, 0]) \
    .setMass(.1) \
    .setContact(True)

bot = ry.BotOp(C, False)
bot.home(C)

openvr.init(openvr.VRApplication_Background)

left_controller_index = openvr.k_unTrackedDeviceIndexInvalid
right_controller_index = openvr.k_unTrackedDeviceIndexInvalid
for i in range(openvr.k_unMaxTrackedDeviceCount):
    device_class = openvr.VRSystem().getTrackedDeviceClass(i)
    if device_class == openvr.TrackedDeviceClass_Controller and openvr.VRSystem().getControllerRoleForTrackedDeviceIndex(i) == openvr.TrackedControllerRole_LeftHand:
        right_controller_index = i
    if device_class == openvr.TrackedDeviceClass_Controller and openvr.VRSystem().getControllerRoleForTrackedDeviceIndex(i) == openvr.TrackedControllerRole_RightHand:
        left_controller_index = i

def getPosition():
    pose = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
    left_pose = pose[left_controller_index].mDeviceToAbsoluteTracking
    return np.array([-left_pose[0][3], left_pose[2][3], left_pose[1][3]])*5


def getRotation():
    pose = openvr.VRSystem().getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
    left_pose = pose[left_controller_index].mDeviceToAbsoluteTracking

    rotation = np.array([
        [left_pose[0][0], left_pose[0][1], left_pose[0][2]],
        [left_pose[1][0], left_pose[1][1], left_pose[1][2]],
        [left_pose[2][0], left_pose[2][1], left_pose[2][2]],
    ])
    

    y_angle, z_angle, x_angle = st.Rotation.from_matrix(rotation).as_euler("xyz", degrees=False)
    

    print( np.array([x_angle, y_angle, z_angle]))
    #sleep(0.5)

    #r = R.from_rotvec([x_angle, y_angle, z_angle])

    #return r.as_matrix()
    return np.array([-z_angle, x_angle, y_angle])

initial_position = getPosition() -C.frame("pen").getPosition()
initial_rotation = getRotation()


while True:

    _, controller = openvr.VRSystem().getControllerState(left_controller_index)
    if bool(controller.ulButtonPressed >> 32 & 1):
        initial_position = getPosition() - C.frame("pen").getPosition()
        initial_rotation = getRotation()
        

    current_position = getPosition()
    current_rotation = getRotation()

    C.frame("marker").setPosition(current_position-initial_position)

    r = R.from_rotvec(current_rotation-initial_rotation).as_matrix()
    C.frame("marker").setQuaternion(R.from_matrix(r).as_quat())


    controller_up_vector = np.dot(r, np.array([0, 1, 0]))
    controller_side_vector = np.dot(r, np.array([0, 0, 1]))
    update_robot(C, current_position-initial_position, bot, controller, controller_up_vector, controller_side_vector)
    bot.sync(C, .1)
    print(bot.getKeyPressed())
    if bot.getKeyPressed()==113:
        break
