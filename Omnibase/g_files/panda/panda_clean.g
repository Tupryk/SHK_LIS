panda_link0: {  }
panda_link0_0(panda_link0): { shape: mesh, mesh: <franka_description/meshes/visual/link0.ply>, visual: True }
panda_joint1_origin(panda_link0): { rel: [0, 0, 0.333, 1, 0, 0, 0] }
panda_joint1(panda_joint1_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] }
panda_link1(panda_joint1): {  }
panda_link1_0(panda_link1): { shape: mesh, mesh: <franka_description/meshes/visual/link1.ply>, visual: True }
panda_joint2_origin(panda_link1): { rel: [0, 0, 0, 0.707107, -0.707107, 0, 0] }
panda_joint2(panda_joint2_origin): { joint: hingeZ, limits: [-1.7628, 1.7628, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] }
panda_link2(panda_joint2): {  }
panda_link2_0(panda_link2): { shape: mesh, mesh: <franka_description/meshes/visual/link2.ply>, visual: True }
panda_joint3_origin(panda_link2): { rel: [0, -0.316, 0, 0.707107, 0.707107, 0, 0] }
panda_joint3(panda_joint3_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] }
panda_link3(panda_joint3): {  }
panda_link3_0(panda_link3): { shape: mesh, mesh: <franka_description/meshes/visual/link3.ply>, visual: True }
panda_joint4_origin(panda_link3): { rel: [0.0825, 0, 0, 0.707107, 0.707107, 0, 0] }
panda_joint4(panda_joint4_origin): { joint: hingeZ, limits: [-3.0718, -0.0698, 2.175, -1, 87], ctrl_limits: [2.175, -1, 87] }
panda_link4(panda_joint4): {  }
panda_link4_0(panda_link4): { shape: mesh, mesh: <franka_description/meshes/visual/link4.ply>, visual: True }
panda_joint5_origin(panda_link4): { rel: [-0.0825, 0.384, 0, 0.707107, -0.707107, 0, 0] }
panda_joint5(panda_joint5_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] }
panda_link5(panda_joint5): {  }
panda_link5_0(panda_link5): { shape: mesh, mesh: <franka_description/meshes/visual/link5.ply>, visual: True }
panda_joint6_origin(panda_link5): { rel: [0, 0, 0, 0.707107, 0.707107, 0, 0] }
panda_joint6(panda_joint6_origin): { joint: hingeZ, limits: [-0.0175, 3.7525, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] }
panda_link6(panda_joint6): {  }
panda_link6_0(panda_link6): { shape: mesh, mesh: <franka_description/meshes/visual/link6.ply>, visual: True }
panda_joint7_origin(panda_link6): { rel: [0.088, 0, 0, 0.707107, 0.707107, 0, 0] }
panda_joint7(panda_joint7_origin): { joint: hingeZ, limits: [-2.8973, 2.8973, 2.61, -1, 12], ctrl_limits: [2.61, -1, 12] }
panda_link7(panda_joint7): {  }
panda_link7_0(panda_link7): { shape: mesh, mesh: <franka_description/meshes/visual/link7.ply>, visual: True }
panda_joint8_origin(panda_link7): { rel: [0, 0, 0.107, 1, 0, 0, 0] }
panda_joint8(panda_joint8_origin): { joint: rigid }
panda_link8(panda_joint8): {  }
panda_hand_joint_origin(panda_link8): { rel: [0, 0, 0, 0.92388, 0, 0, -0.382683] }
panda_hand_joint(panda_hand_joint_origin): { joint: rigid }
panda_hand(panda_hand_joint): {  }
panda_hand_0(panda_hand): { shape: mesh, mesh: <franka_description/meshes/visual/hand.ply>, visual: True }
panda_finger_joint1_origin(panda_hand): { Q: [0, 0, 0.0584, 1, 0, 0, 0] }
panda_finger_joint2_origin(panda_hand): { Q: [0, 0, 0.0584, 1, 0, 0, 0] }
panda_finger_joint1(panda_finger_joint1_origin): { joint: transY, limits: [0, 0.04, 0.2, -1, 20], ctrl_limits: [0.2, -1, 20] }
panda_finger_joint2(panda_finger_joint2_origin): { joint: transY, joint_scale: -1, limits: [0, 0.04, 0.2, -1, 20], mimic: "panda_finger_joint1", ctrl_limits: [0.2, -1, 20] }
panda_leftfinger(panda_finger_joint1): {  }
panda_rightfinger(panda_finger_joint2): {  }
panda_leftfinger_0(panda_leftfinger): { shape: mesh, mesh: <franka_description/meshes/visual/finger.ply>, visual: True }
panda_rightfinger_0(panda_rightfinger): { Q: [0, 0, 0, -1.03412e-13, 0, 0, 1], shape: mesh, mesh: <franka_description/meshes/visual/finger.ply>, visual: True }
