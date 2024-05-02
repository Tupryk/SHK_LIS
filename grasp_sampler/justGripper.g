panda_hand: {  }
panda_hand_0(panda_hand): { shape: mesh, color: [0.9], mesh: <franka_description/meshes/visual/hand.ply>, visual: True }
panda_finger_joint1_origin(panda_hand): { Q: [0, 0, 0.0584, 1, 0, 0, 0] }
panda_finger_joint2_origin(panda_hand): { Q: [0, 0, 0.0584, 1, 0, 0, 0] }
panda_finger_joint1(panda_finger_joint1_origin): { joint: transY, limits: [0, 0.04, 0.2, -1, 20], ctrl_limits: [0.2, -1, 20] }
panda_finger_joint2(panda_finger_joint2_origin): { joint: transY, joint_scale: -1, limits: [0, 0.04, 0.2, -1, 20], mimic: "panda_finger_joint1", ctrl_limits: [0.2, -1, 20] }
panda_leftfinger(panda_finger_joint1): {  }
panda_rightfinger(panda_finger_joint2): {  }
panda_leftfinger_0(panda_leftfinger): { shape: mesh, color: [0.9], mesh: <franka_description/meshes/visual/finger.ply>, visual: True }
panda_rightfinger_0(panda_rightfinger): { Q: [0, 0, 0, -1.03412e-13, 0, 0, 1], shape: mesh, color: [0.9], mesh: <franka_description/meshes/visual/finger.ply>, visual: True }
