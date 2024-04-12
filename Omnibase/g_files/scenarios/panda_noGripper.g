## this should be the default panda we use on the real system
# with NO dofs for the gripper

Include: <../panda/panda.g>

# modify default home pose

Edit panda_joint2: { q: -.5 }
Edit panda_joint4: { q: -2 }
Edit panda_joint7: { q: -.5 }

# delete original gripper

Delete panda_hand_joint_origin:
Delete panda_hand_joint:
Delete panda_hand_0:
Delete panda_finger_joint1_origin:
Delete panda_finger_joint2_origin:
Delete panda_finger_joint1:
Delete panda_finger_joint2:
Delete panda_leftfinger_0:
Delete panda_rightfinger_0:
#Delete panda_coll_hand:
#Delete panda_coll_finger1:
#Delete panda_coll_finger2:

Delete gripper:
Delete palm:
Delete finger1:
Delete finger2:
