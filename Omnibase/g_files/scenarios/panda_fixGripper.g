## this should be the default panda we use on the real system
# with NO dofs for the gripper

Include: <../panda/panda.g>

# make fingers part of the gripper link
#Edit panda_finger_joint1: { Q: "t(0 .05 0)", joint: none }
#Edit panda_finger_joint2: { Q: "t(0 -.05 0)", joint: none }

Edit panda_finger_joint1: { joint_active: False }
 
