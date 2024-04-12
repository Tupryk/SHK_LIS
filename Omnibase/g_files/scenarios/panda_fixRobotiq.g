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
 
# add robotiq
Include: <../robotiq/robotiq.g>
Edit robotiq_base (panda_joint8): { Q: [0, 0, .050] }

Edit panda_coll7: { Q: "t(0 .0 .07)", size: [.22, .07] }
 
# add extra collision object for panda wrist
panda_coll7b(panda_joint7): { shape: capsule, color: [1.,1.,1.,.2], size: [.05, .05], Q: "d(-90 0 1 0) t(.08 .0 .0) d(45 1 0 0) t(0 0 -.03)", contact: -2 }
 
# kill panda 8th joint
Edit panda_joint8: { joint: none }

# make finger_joint non-active
Edit finger_joint: { joint_active: false }

#Edit finger_joint: { joint: none }
#Edit left_inner_knuckle_joint: { joint: none }
##Edit left_outer_knuckle_joint: { joint: none }
#Edit right_inner_knuckle_joint: { joint: none }
#Edit right_outer_knuckle_joint: { joint: none }

#Edit right_inner_finger_joint: { joint: none, limits: [-.042, .0] }
#Edit left_inner_finger_joint: { joint: none, limits: [-.042, .0] }
