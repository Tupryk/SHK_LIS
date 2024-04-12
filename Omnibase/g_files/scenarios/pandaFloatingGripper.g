base { multibody }

floatX (base){ joint:transX, limits:[-2 2], mass:.01 }
floatY (floatX){ joint:transY, limits:[-2 2], mass:.01 }
floatZ (floatY){ joint:transZ, limits:[0 3], mass:.01, q: 1 }
floatBall (floatZ){ joint:quatBall, limits:[-1 1 -1 1 -1 1 -1 1], mass:.01 }

Include: <../panda/panda_gripper.g>

gripper_base(floatBall): { Q:"t(0 0 .1035) d(180 1 0 0) d(-90 0 0 1)", shape: marker, size: [.1] }
Edit panda_hand(gripper_base): {}

## define a gripper, palm and fingers

gripper(panda_hand): { Q: "d(180 0 1 0) d(90 0 0 1) t(0 0 -.1035)", shape: marker, size: [.03], color: [.9, .9, .9], logical: { is_gripper } }
palm(panda_hand): { Q: "d(90 1 0 0)", shape: capsule, color: [1.,1.,1.,.2], size: [.14, .07], contact: -3 }
finger1(panda_finger_joint1): { Q: [0, 0.028, .035], shape: capsule, size: [.02, .03], color: [1., 1., 1., .2], contact: -2 }
finger2(panda_finger_joint2): { Q: [0, -.028, .035], shape: capsule, size: [.02, .03], color: [1., 1., 1., .2], contact: -2 }

dotA(panda_finger_joint1){ Q:[0, 0, .0451], shape:sphere, size:[.01], color:[1 0 0 .5] }
dotB(panda_finger_joint2){ Q:[0, 0, .0451], shape:sphere, size:[.01], color:[1 0 0 .5] }

Edit panda_finger_joint1: { q: .04 }
