base_origin: { X: [0, 0, .2] }

base (base_origin): {
 joint: transXYPhi, limits: [-10,10,-10,10,-4,4],
#, shape: ssCvx, mesh: <base.arr>, meshscale: .3, size: [.02], contact: 1 }
 shape: ssCylinder, size: [.1, .3, .02], contact: 1 }

Include: <panda_fixGripper.g>

Edit panda_base (base): { Q: "t(0 0 .05) d(90 0 0 1)" }

# add E.T. hooks
hookA (panda_joint5): { Q: [0, .2, 0], shape: sphere, size: [.025], color: [.6, .9, .9] }
(hookA): { Q: [0, -.05, 0, 1, 1, 0, 0], shape: capsule, size: [.075, .01], color: [.9, .9, .9] }

hookB (gripper): { Q: [.15, 0, .02], shape: sphere, size: [.025], color: [.6, .9, .9] }
(hookB): { Q: [-.05, 0, 0, 1, 0, 1, 0], shape: capsule, size: [.075, .01], color: [.9, .9, .9] }
