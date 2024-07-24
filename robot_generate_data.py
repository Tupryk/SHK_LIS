import h5py
import rowan
import numpy as np
import robotic as ry
import manipulation as manip
import matplotlib.pyplot as plt


EXAMPLE_COUNT = 50
count = 0
for i in range(EXAMPLE_COUNT):
    cam1s = []
    cam2s = []
    qpos_data = []
    action_data = []

    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

    x_pos = np.random.random() * .2 - .2
    y_pos = np.random.random() * .1 + .2

    C.addFrame("cam1")\
        .setShape(ry.ST.camera, [.1]) \
        .setPosition([0., .8, 1.5]) \
        .setQuaternion(rowan.from_euler(0, 0, np.pi*.75))\
        .addAttributes({'focalLength':0.895, 'width':320., 'height':180.})
    
    C.addFrame("cam2")\
        .setShape(ry.ST.camera, [.1]) \
        .setPosition([-.8, .5, 1.]) \
        .setQuaternion(rowan.from_euler(np.pi*.5, np.pi*.5, np.pi*.75))\
        .addAttributes({'focalLength':0.895, 'width':320., 'height':180.})
    
    C.addFrame("box", "table") \
        .setJoint(ry.JT.rigid) \
        .setShape(ry.ST.ssBox, [.15,.06,.06,.005]) \
        .setRelativePosition([x_pos, y_pos, .095]) \
        .setContact(1) \
        .setMass(.1)
    
    C.addFrame("goal", "table") \
        .setShape(ry.ST.ssBox, [.1, .1, .02, .005]) \
        .setPosition([.2, .1, .65]) \
        .setColor([1, 0, 0]) \
        .setContact(1)

    C.delFrame("panda_collCameraWrist")

    # for convenience, a few definitions:
    qHome = C.getJointState()
    gripper = "l_gripper"
    palm = "l_palm"
    box = "box"
    table = "table"
    boxSize = C.frame(box).getSize()

    C.setJointState(qHome)

    qStart = C.getJointState()

    graspDirection = 'yz'
    placeDirection = 'z'
    place_orientation = [-(i%2),((i+1)%2),0.]

    M = manip.ManipulationModelling(C, helpers=[gripper])
    M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1, joint_limits=False)
    M.grasp_top_box(1., gripper, box, graspDirection)
    M.place_box(2., box, table, palm, placeDirection)
    M.target_relative_xy_position(2., box, "goal", [0, 0])
    M.target_x_orientation(2., box, place_orientation)
    M.solve()
    if not M.feasible:
        continue

    M2 = M.sub_motion(0)
    M2.retract([.0, .2], gripper)
    M2.approach([.8, 1.], gripper)
    M2.solve()
    if not M2.ret.feasible:
        continue

    M3 = M.sub_motion(1)
    M3.no_collision([], table, box)
    M3.bias(.5, qHome, 1e0)
    M3.solve()
    if not M3.ret.feasible:
        continue
    

    bot = ry.BotOp(C, False)
    for t in range(1, M2.path.shape[0]):
        C.setJointState(M2.path[t-1])
        qpos_data.append(M2.path[t-1])
        action_data.append(M2.path[t])
        cam1, _, _ = bot.getImageDepthPcl('cam1')
        cam2, _, _ = bot.getImageDepthPcl('cam2')
        cam1s.append(cam1)
        cam2s.append(cam2)

    C.attach(gripper, box)

    for t in range(1, M3.path.shape[0]):
        C.setJointState(M3.path[t-1])
        qpos_data.append(M3.path[t-1])
        action_data.append(M3.path[t])
        cam1, _, _ = bot.getImageDepthPcl('cam1')
        cam2, _, _ = bot.getImageDepthPcl('cam2')
        cam1s.append(cam1)
        cam2s.append(cam2)

    C.attach(table, box)

    with h5py.File(f"../../awe/dataset/episode_{count}.hdf5", "w") as hdf:
        hdf.attrs["sim"] = True
        
        hdf.create_dataset("/action", data=action_data)
        
        observations_group = hdf.create_group("/observations")
        observations_group.create_dataset("qpos", data=qpos_data)
        observations_group.create_dataset("qvel", data=[])
        
        images_group = observations_group.create_group("images")
        images_group.create_dataset("cam1", data=cam1s)
        images_group.create_dataset("cam2", data=cam2s)
        
        hdf.create_dataset("/waypoints", data=[])
    
    count += 1
